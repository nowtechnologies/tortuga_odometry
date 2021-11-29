#include <tortuga_odometry/odom_estimation_node.h>

using namespace MatrixWrapper;
using namespace std;
using namespace ros;
using namespace tf;


static const double EPS = 1e-5;


//#define __EKF_DEBUG_FILE__

namespace estimation
{
  // Utils
  template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

  // constructor
  OdomEstimationNode::OdomEstimationNode()
    : odom_active_(false),
      imu_active_(false),
      vo_active_(false),
      gps_active_(false),
      odom_initializing_(false),
      imu_initializing_(false),
      vo_initializing_(false),
      gps_initializing_(false),
      odom_covariance_(6),
      imu_covariance_(3),
      vo_covariance_(6),
      gps_covariance_(3),
      odom_callback_counter_(0),
      imu_callback_counter_(0),
      vo_callback_counter_(0),
      gps_callback_counter_(0),
      ekf_sent_counter_(0)
  {
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    // paramters
    nh_private.param("output_frame", output_frame_, std::string("odom_combined"));
    nh_private.param("base_footprint_frame", base_footprint_frame_, std::string("base_footprint"));
    nh_private.param("sensor_timeout", timeout_, 1.0);
    nh_private.param("odom_used", odom_used_, true);
    nh_private.param("imu_used",  imu_used_, true);
    nh_private.param("vo_used",   vo_used_, true);
    nh_private.param("gps_used",   gps_used_, false);
    nh_private.param("debug",   debug_, false);
    nh_private.param("publish_transform",   publish_transform_, true);
    nh_private.param("self_diagnose",  self_diagnose_, false);
    nh_private.param("wheel_odometry_validation", wheel_odometry_validation_enabled, false);
    nh_private.param("wheel_odometry_reliability_treshold", wheel_odometry_reliability_treshold, 0.5);
    nh_private.param("wheel_odometry_validation_debug", wheel_odometry_validation_debug, true);
    double freq;
    nh_private.param("freq", freq, 30.0);

    tf_prefix_ = tf::getPrefixParam(nh_private);
    output_frame_ = tf::resolve(tf_prefix_, output_frame_);
    base_footprint_frame_ = tf::resolve(tf_prefix_, base_footprint_frame_);

    // Initialize wheel-odometry validator variables
    wheel_odometry_reliability = 1.0;

    ROS_INFO_STREAM("output frame: " << output_frame_);
    ROS_INFO_STREAM("base frame: " << base_footprint_frame_);

    // set output frame and base frame names in OdomEstimation filter
    // so that user-defined tf frames are respected
    my_filter_.setOutputFrame(output_frame_);
    my_filter_.setBaseFootprintFrame(base_footprint_frame_);

    timer_ = nh_private.createTimer(ros::Duration(1.0/max(freq,1.0)), &OdomEstimationNode::spin, this);

    // advertise our estimation
    pose_pub_ = nh_private.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ekf/pose_combined", 10);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/ekf/odom", 10);
    // advertise wheelodom reliability
    wheel_odometry_reliability_pub = nh.advertise<std_msgs::Float32>("/ekf/odom_reliability", 10);
    imu_angular_velocity_base = tf::Vector3(0,0,0);
    imu_linear_acceleration_base_previous = tf::Vector3(0,0,0);
    if (wheel_odometry_validation_debug) {
      debug_twist_imu_pub  = nh.advertise<geometry_msgs::Twist>("/ekf/debug/twist_imu", 10);
      debug_twist_odom_pub = nh.advertise<geometry_msgs::Twist>("/ekf/debug/twist_odom", 10);
    }

    // initialize
    filter_stamp_ = Time::now();

    // subscribe to odom messages
    if (odom_used_){
      ROS_DEBUG("Odom sensor can be used");
      odom_sub_ = nh.subscribe("odom", 10, &OdomEstimationNode::odomCallback, this);
    }
    else ROS_DEBUG("Odom sensor will NOT be used");

    // subscribe to imu messages
    if (imu_used_){
      ROS_DEBUG("Imu sensor can be used");
      imu_sub_ = nh.subscribe("imu_data", 10,  &OdomEstimationNode::imuCallback, this);
    }
    else ROS_DEBUG("Imu sensor will NOT be used");

    // subscribe to vo messages
    if (vo_used_){
      ROS_DEBUG("VO sensor can be used");
      vo_sub_ = nh.subscribe("vo", 10, &OdomEstimationNode::voCallback, this);
    }
    else ROS_DEBUG("VO sensor will NOT be used");

    if (gps_used_){
      ROS_DEBUG("GPS sensor can be used");
      gps_sub_ = nh.subscribe("gps", 10, &OdomEstimationNode::gpsCallback, this);
    }
    else ROS_DEBUG("GPS sensor will NOT be used");

    if (debug_){
      // open files for debugging
      odom_file_.open("/tmp/odom_file.txt");
      imu_file_.open("/tmp/imu_file.txt");
      vo_file_.open("/tmp/vo_file.txt");
      gps_file_.open("/tmp/gps_file.txt");
      corr_file_.open("/tmp/corr_file.txt");
    }
  };

  // destructor
  OdomEstimationNode::~OdomEstimationNode(){

    if (debug_){
      // close files for debugging
      odom_file_.close();
      imu_file_.close();
      gps_file_.close();
      vo_file_.close();
      corr_file_.close();
    }
  };


  // callback function for odom data

  void OdomEstimationNode::odomCallback(const OdomConstPtr& odom)
  {
    odom_callback_counter_++;

    ROS_DEBUG("Odom callback at time %f ", ros::Time::now().toSec());
    assert(odom_used_);

    // receive data
    odom_stamp_  = odom->header.stamp;
    odom_time_   = Time::now();
    wheelTwist_  = odom->twist;
    Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    odom_meas_  = Transform(q, Vector3(odom->pose.pose.position.x, odom->pose.pose.position.y, 0));
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        odom_covariance_(i+1, j+1) = odom->pose.covariance[6*i+j];

    if (wheel_odometry_validation_enabled) {
      validateWheelOdometry(odom);
    } else {
      // Odometry covariance unchanged
    }
    bool wheel_odom_valid = (wheel_odometry_reliability >= wheel_odometry_reliability_treshold);
    // Deactivate odom for the current update
    if (!wheel_odom_valid) { odom_active_ = false; }

    my_filter_.addMeasurement(StampedTransform(odom_meas_.inverse(), odom_stamp_, base_footprint_frame_, "wheelodom"), odom_covariance_);

    // activate odom only if it is valid
    if (!odom_active_ && wheel_odom_valid) {
      if (!odom_initializing_){
      	odom_initializing_ = true;
      	odom_init_stamp_ = odom_stamp_;
      	ROS_INFO("Initializing Odom sensor");
      }
      if ( filter_stamp_ >= odom_init_stamp_){
        odom_active_ = true;
        odom_initializing_ = false;
        ROS_INFO("Odom sensor activated");
      }
      else ROS_DEBUG("Waiting to activate Odom, because Odom measurements are still %f sec in the future.",
		    (odom_init_stamp_ - filter_stamp_).toSec());
    }

    if (debug_){
      // write to file
      double tmp, yaw;
      odom_meas_.getBasis().getEulerYPR(yaw, tmp, tmp);
      odom_file_<< fixed <<setprecision(5) << ros::Time::now().toSec() << " " << odom_meas_.getOrigin().x() << " " << odom_meas_.getOrigin().y() << "  " << yaw << "  " << endl;
    }
  };




  // callback function for imu data
  void OdomEstimationNode::imuCallback(const ImuConstPtr& imu)
  {
    imu_callback_counter_++;

    assert(imu_used_);

    // receive data
    imu_stamp_ = imu->header.stamp;
    tf::Quaternion orientation;
    quaternionMsgToTF(imu->orientation, orientation);
    imu_meas_ = tf::Transform(orientation, tf::Vector3(0,0,0));
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        imu_covariance_(i+1, j+1) = imu->orientation_covariance[3*i+j];

    // Transforms imu data to base_footprint frame
    if (!robot_state_.waitForTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, ros::Duration(0.5))){
      // warn when imu was already activated, not when imu is not active yet
      if (imu_active_)
        ROS_ERROR("Could not transform imu message from %s to %s", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      else if (my_filter_.isInitialized())
        ROS_WARN("Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      else
        ROS_DEBUG("Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      return;
    }
    StampedTransform base_imu_offset;
    robot_state_.lookupTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, base_imu_offset);
    imu_meas_ = imu_meas_ * base_imu_offset;

    // Save previous time
    imu_time_prev_ = imu_time_;
    // Get current time
    imu_time_  = Time::now();

    // Save previous accel and vel before updating them now
    imu_linear_acceleration_base_previous = imu_linear_acceleration_base;
    imu_angular_velocity_base_previous    = imu_angular_velocity_base;

    // Extract raw sensor values from imu
    imu_angular_velocity    = imu->angular_velocity;
    imu_linear_acceleration = imu->linear_acceleration;
    // Transform raw sensor values to base_footprint frame, this is a pure geometric rotation only
    tf::Matrix3x3 base_imu_offset_mx(base_imu_offset.getRotation());
    tf::Vector3   base_imu_vector(base_imu_offset.getOrigin()); // TODO: Is this really the displacement?
    // Angular velocity in base frame
    imu_angular_velocity_base = base_imu_offset_mx * tf::Vector3(imu_angular_velocity.x, imu_angular_velocity.y, imu_angular_velocity.z);;
    // Estimate angular acceleration (in base frame)
    imu_angular_acceleration_base = (imu_angular_velocity_base-imu_angular_velocity_base_previous)/((imu_time_-imu_time_prev_).toSec()); // rad/s^2
    // Linear acceleration at the IMU (not at the BASE frame origin!) in the base frame reference system
    imu_linear_acceleration_base = base_imu_offset_mx * tf::Vector3(imu_linear_acceleration.x, imu_linear_acceleration.y, imu_linear_acceleration.z);;
    // "Translate" linear accel to the base frame
    // aB = aI + eps x rIB + om x (om x rIB)
    // where aI is the acceleration at point IMU in base_frame
    //       aB is the acceleration at point BASE in base_frame
    //       eps is the angular acceleration in base_frame
    //       om is the angular acceleration in base_frame
    //       rIB is the vector between points IMU and BASE in base_frame
    // at this point imu_linear_acceleration_base = aI
    imu_linear_acceleration_base = imu_linear_acceleration_base
                                + imu_angular_acceleration_base.cross(base_imu_vector)
                                + imu_angular_velocity_base.cross(imu_angular_velocity_base.cross(base_imu_vector));
    // Now imu_linear_acceleration_base = aB

    // manually set covariance untile imu sends covariance
    if (imu_covariance_(1,1) == 0.0){
      SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
      measNoiseImu_Cov(1,1) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(2,2) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(3,3) = pow(0.00017,2);  // = 0.01 degrees / sec
      imu_covariance_ = measNoiseImu_Cov;
    }

    my_filter_.addMeasurement(StampedTransform(imu_meas_.inverse(), imu_stamp_, base_footprint_frame_, "imu"), imu_covariance_);

    // activate imu
    if (!imu_active_) {
      if (!imu_initializing_){
        imu_initializing_ = true;
        imu_init_stamp_ = imu_stamp_;
        ROS_INFO("Initializing Imu sensor");
      }
      if ( filter_stamp_ >= imu_init_stamp_){
        imu_active_ = true;
        imu_initializing_ = false;
        ROS_INFO("Imu sensor activated");
      }
      else ROS_DEBUG("Waiting to activate IMU, because IMU measurements are still %f sec in the future.",
		    (imu_init_stamp_ - filter_stamp_).toSec());
    }

    if (debug_){
      // write to file
      double tmp, yaw;
      imu_meas_.getBasis().getEulerYPR(yaw, tmp, tmp);
      imu_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< yaw << endl;
    }
  };




  // callback function for VO data
  void OdomEstimationNode::voCallback(const VoConstPtr& vo)
  {
    vo_callback_counter_++;

    assert(vo_used_);

    // get data
    vo_stamp_ = vo->header.stamp;
    vo_time_  = Time::now();
    poseMsgToTF(vo->pose.pose, vo_meas_);
    for (unsigned int i=0; i<6; i++)
      for (unsigned int j=0; j<6; j++)
        vo_covariance_(i+1, j+1) = vo->pose.covariance[6*i+j];
    my_filter_.addMeasurement(StampedTransform(vo_meas_.inverse(), vo_stamp_, base_footprint_frame_, "vo"), vo_covariance_);

    // activate vo
    if (!vo_active_) {
      if (!vo_initializing_){
        vo_initializing_ = true;
        vo_init_stamp_ = vo_stamp_;
        ROS_INFO("Initializing Vo sensor");
      }
      if (filter_stamp_ >= vo_init_stamp_){
        vo_active_ = true;
        vo_initializing_ = false;
        ROS_INFO("Vo sensor activated");
      }
      else ROS_DEBUG("Waiting to activate VO, because VO measurements are still %f sec in the future.",
		    (vo_init_stamp_ - filter_stamp_).toSec());
    }

    if (debug_){
      // write to file
      double Rx, Ry, Rz;
      vo_meas_.getBasis().getEulerYPR(Rz, Ry, Rx);
      vo_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< vo_meas_.getOrigin().x() << " " << vo_meas_.getOrigin().y() << " " << vo_meas_.getOrigin().z() << " "
               << Rx << " " << Ry << " " << Rz << endl;
    }
  };


  void OdomEstimationNode::gpsCallback(const GpsConstPtr& gps)
  {
    gps_callback_counter_++;

    assert(gps_used_);

    // get data
    gps_stamp_ = gps->header.stamp;
    gps_time_  = Time::now();
    poseMsgToTF(gps->pose.pose, gps_meas_);
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        gps_covariance_(i+1, j+1) = gps->pose.covariance[6*i+j];
    my_filter_.addMeasurement(StampedTransform(gps_meas_.inverse(), gps_stamp_, base_footprint_frame_, "gps"), gps_covariance_);

    // activate gps
    if (!gps_active_) {
      if (!gps_initializing_){
	      gps_initializing_ = true;
	      gps_init_stamp_ = gps_stamp_;
	      ROS_INFO("Initializing GPS sensor");
      }
      if (filter_stamp_ >= gps_init_stamp_){
	      gps_active_ = true;
	      gps_initializing_ = false;
	      ROS_INFO("GPS sensor activated");
      }
      else ROS_DEBUG("Waiting to activate GPS, because GPS measurements are still %f sec in the future.",
		    (gps_init_stamp_ - filter_stamp_).toSec());
    }

    if (debug_){
      // write to file
      gps_file_ <<fixed<<setprecision(5)<<ros::Time::now().toSec()<<" "<< gps_meas_.getOrigin().x() << " " << gps_meas_.getOrigin().y() << " " << gps_meas_.getOrigin().z() <<endl;
    }
  };

  void OdomEstimationNode::validateWheelOdometry(const OdomConstPtr& odom) {
    // Notes: wheelTwist_ already contains the twist from odom in child_frame_id
    //        odom_covariance_ contains odom covariance
    geometry_msgs::Vector3 odom_linear_vel  = wheelTwist_.twist.linear;
    geometry_msgs::Vector3 odom_angular_vel = wheelTwist_.twist.angular;
    geometry_msgs::Twist imu_twist;

    // Integrate acceleration with trapezoid rule
    double imu_dt_sec = (imu_time_-imu_time_prev_).toSec();
    // Relative velocity
    double imu_linear_vel_x = (imu_linear_acceleration_base.x() + imu_linear_acceleration_base_previous.x())*imu_dt_sec/2.0;
    double imu_linear_vel_y = (imu_linear_acceleration_base.y() + imu_linear_acceleration_base_previous.y())*imu_dt_sec/2.0;
    double imu_linear_vel_z = (imu_linear_acceleration_base.z() + imu_linear_acceleration_base_previous.z())*imu_dt_sec/2.0;
    // Absolute velocity
    imu_linear_velocity_base += tf::Vector3(imu_linear_vel_x, imu_linear_vel_y, imu_linear_vel_z);
    // Reset velocities to zero if certain conditions are met
    double zero_veloc = 0.01; // m/s
    if (fabs(odom_linear_vel.x) < zero_veloc) imu_linear_velocity_base.setX(0);
    if (fabs(odom_linear_vel.y) < zero_veloc) imu_linear_velocity_base.setY(0);
    if (fabs(odom_linear_vel.z) < zero_veloc) imu_linear_velocity_base.setZ(0);
    // Set debug messages
    if (wheel_odometry_validation_debug) {
      imu_twist.linear.x = imu_linear_velocity_base.x();
      imu_twist.linear.y = imu_linear_velocity_base.y();
      imu_twist.linear.z = imu_linear_velocity_base.z();
      imu_twist.angular.x = imu_angular_velocity_base.x();
      imu_twist.angular.y = imu_angular_velocity_base.y();
      imu_twist.angular.z = imu_angular_velocity_base.z();
    }

    // Calculate angular velocity diff between gyro and odom
    double angular_vel_diff_treshold = 0.15; // 0.1 rad/sec is approx 6 deg/sec, 0.15 rad/s = 9deg/s
    double angular_vel_diff = 0;
    double abs_diff = 0;
    /// TODO: Only Z axis only or check all axes?
    /// Note: temporarily disabled this check
    /* abs_diff = fabs(odom_angular_vel.x - imu_angular_velocity_base.x());
    if (abs_diff > angular_vel_diff_treshold) { angular_vel_diff += abs_diff; }
    abs_diff = fabs(odom_angular_vel.y - imu_angular_velocity_base.y());
    if (abs_diff > angular_vel_diff_treshold) { angular_vel_diff += abs_diff; }
    abs_diff = fabs(odom_angular_vel.z - imu_angular_velocity_base.z());
    if (abs_diff > angular_vel_diff_treshold) { angular_vel_diff += abs_diff; } */

    // Calculate linear velocity diff between integrated accel and wheel odometry
    // The absolute value treshold is a bit steep, but it will also be mixed with a sign check
    double linear_vel_diff_treshold = 0.25; // m/s
    double linear_vel_diff = 0;
    /// TODO: Only X axis (forward) only or check all axes?
    abs_diff = fabs(odom_linear_vel.x - imu_linear_velocity_base.x());
    if (abs_diff > linear_vel_diff_treshold) { linear_vel_diff += abs_diff; }
    abs_diff = fabs(odom_linear_vel.y - imu_linear_velocity_base.y());
    if (abs_diff > linear_vel_diff_treshold) { linear_vel_diff += abs_diff; }
    abs_diff = fabs(odom_linear_vel.z - imu_linear_velocity_base.z());
    if (abs_diff > linear_vel_diff_treshold) { linear_vel_diff += abs_diff; }

    // Sign check on forward velocity
    double sign_check_treshold = 0.1;
    if (fabs(odom_linear_vel.x) > sign_check_treshold) {
       if (sgn(odom_linear_vel.x) != sgn(imu_linear_velocity_base.x())) {
        linear_vel_diff += linear_vel_diff_treshold;  // Pulls down reliability
      }
    }

    // TODO? Relative velocity tolerance

    // Update reliability value
    if ((angular_vel_diff > 0) || (linear_vel_diff > 0)) {
      wheel_odometry_reliability = 1.0 - angular_vel_diff/angular_vel_diff_treshold - linear_vel_diff/linear_vel_diff_treshold;
      if (wheel_odometry_reliability <= 0.0001) wheel_odometry_reliability = 0.0001;
    } else {
      wheel_odometry_reliability = 1.0;
    }

    // Modify odom_covariance: multiply by 1/wheel_odom_reliability
    double cov_magn = 1.0/wheel_odometry_reliability;
    for (unsigned int i=0; i<6; i++) {
      for (unsigned int j=0; j<6; j++) odom_covariance_(i+1, j+1) = cov_magn*odom->pose.covariance[6*i+j];
    }

    // Publish message(s)
    wheel_odometry_reliability_msg.data = wheel_odometry_reliability;
    wheel_odometry_reliability_pub.publish(wheel_odometry_reliability_msg);

    // Debug messages
    if (wheel_odometry_validation_debug) {
      ROS_INFO_STREAM("Wheel odom reliability = " << wheel_odometry_reliability << ", covariance magnification: " << (int)cov_magn);
      debug_twist_imu_pub.publish(imu_twist);
      debug_twist_odom_pub.publish(wheelTwist_.twist);
    }
  }

  // filter loop
  void OdomEstimationNode::spin(const ros::TimerEvent& e)
  {
    ROS_DEBUG("Spin function at time %f", ros::Time::now().toSec());

    // check for timing problems
    if ( (odom_initializing_ || odom_active_) && (imu_initializing_ || imu_active_) ){
      double diff = fabs( Duration(odom_stamp_ - imu_stamp_).toSec() );
      if (diff > 1.0) ROS_ERROR("Timestamps of odometry and imu are %f seconds apart.", diff);
    }

    // initial value for filter stamp; keep this stamp when no sensors are active
    filter_stamp_ = Time::now();

    // check which sensors are still active
    if ((odom_active_ || odom_initializing_) &&
        (Time::now() - odom_time_).toSec() > timeout_){
      odom_active_ = false; odom_initializing_ = false;
      ROS_INFO("Odom sensor not active any more");
    }
    if ((imu_active_ || imu_initializing_) &&
        (Time::now() - imu_time_).toSec() > timeout_){
      imu_active_ = false;  imu_initializing_ = false;
      ROS_INFO("Imu sensor not active any more");
    }
    if ((vo_active_ || vo_initializing_) &&
        (Time::now() - vo_time_).toSec() > timeout_){
      vo_active_ = false;  vo_initializing_ = false;
      ROS_INFO("VO sensor not active any more");
    }

    if ((gps_active_ || gps_initializing_) &&
        (Time::now() - gps_time_).toSec() > timeout_){
      gps_active_ = false;  gps_initializing_ = false;
      ROS_INFO("GPS sensor not active any more");
    }


    // only update filter when one of the sensors is active
    if (odom_active_ || imu_active_ || vo_active_ || gps_active_){

      // update filter at time where all sensor measurements are available
      if (odom_active_)  filter_stamp_ = min(filter_stamp_, odom_stamp_);
      if (imu_active_)   filter_stamp_ = min(filter_stamp_, imu_stamp_);
      if (vo_active_)    filter_stamp_ = min(filter_stamp_, vo_stamp_);
      if (gps_active_)  filter_stamp_ = min(filter_stamp_, gps_stamp_);


      // update filter
      if ( my_filter_.isInitialized() )  {
        bool diagnostics = true;
        if (my_filter_.update(odom_active_, imu_active_,gps_active_, vo_active_,  filter_stamp_, diagnostics)){

          // output most recent estimate and relative covariance
          my_filter_.getEstimate(output_);

          odom_output_.header.stamp = output_.header.stamp;
          odom_output_.header.frame_id = output_frame_;
          odom_output_.child_frame_id  = base_footprint_frame_;
          odom_output_.pose  = output_.pose;
          odom_output_.twist = wheelTwist_;
          odom_pub_.publish(odom_output_);

          pose_pub_.publish(output_);
          ekf_sent_counter_++;

          // broadcast most recent estimate to TransformArray
          StampedTransform tmp;
          my_filter_.getEstimate(ros::Time(), tmp);
          if(!vo_active_ && !gps_active_)
            tmp.getOrigin().setZ(0.0);
          if(publish_transform_)
            odom_broadcaster_.sendTransform(StampedTransform(tmp, tmp.stamp_, output_frame_, base_footprint_frame_));

          if (debug_){
            // write to file
            ColumnVector estimate;
            my_filter_.getEstimate(estimate);
            corr_file_ << fixed << setprecision(5)<<ros::Time::now().toSec()<<" ";

            for (unsigned int i=1; i<=6; i++)
            corr_file_ << estimate(i) << " ";
            corr_file_ << endl;
          }
        }
        if (self_diagnose_ && !diagnostics)
          ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
      }


      // initialize filer with odometry frame
      if (imu_active_ && gps_active_ && !my_filter_.isInitialized()) {
        Quaternion q = imu_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps and imu measurement");
      }
      else if ( odom_active_ && gps_active_ && !my_filter_.isInitialized()) {
        Quaternion q = odom_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps and odometry measurement");
      }
      else if ( vo_active_ && gps_active_ && !my_filter_.isInitialized()) {
        Quaternion q = vo_meas_.getRotation();
        Vector3 p = gps_meas_.getOrigin();
        Transform init_meas_ = Transform(q, p);
        my_filter_.initialize(init_meas_, gps_stamp_);
        ROS_INFO("Kalman filter initialized with gps and visual odometry measurement");
      }
      else if ( odom_active_  && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(odom_meas_, odom_stamp_);
        ROS_INFO("Kalman filter initialized with odom measurement");
      }
      else if ( vo_active_ && !gps_used_ && !my_filter_.isInitialized()){
        my_filter_.initialize(vo_meas_, vo_stamp_);
        ROS_INFO("Kalman filter initialized with vo measurement");
      }
    }
  };

}; // namespace






// ----------
// -- MAIN --
// ----------
using namespace estimation;
int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "robot_pose_ekf");

  // create filter class
  OdomEstimationNode my_filter_node;

  ros::spin();

  return 0;
}
