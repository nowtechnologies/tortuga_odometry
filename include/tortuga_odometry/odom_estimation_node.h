#ifndef __ODOM_ESTIMATION_NODE__
#define __ODOM_ESTIMATION_NODE__

// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "odom_estimation.h"

// messages
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Float32.h"

#include <boost/thread/mutex.hpp>

// log files
#include <fstream>

namespace estimation
{

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> VoConstPtr;
typedef boost::shared_ptr<nav_msgs::Odometry const> GpsConstPtr;
typedef boost::shared_ptr<geometry_msgs::Twist const> VelConstPtr;

class OdomEstimationNode
{
public:
  /// constructor
  OdomEstimationNode();

  /// destructor
  virtual ~OdomEstimationNode();

private:
  /// the mail filter loop that will be called periodically
  void spin(const ros::TimerEvent& e);

  /// callback function for odo data
  void odomCallback(const OdomConstPtr& odom);

  /// callback function for imu data
  void imuCallback(const ImuConstPtr& imu);

  /// callback function for vo data
  void voCallback(const VoConstPtr& vo);

  /// callback function for vo data
  void gpsCallback(const GpsConstPtr& gps);

  /// calculate wheel odometry reliability
  void validateWheelOdometry(const OdomConstPtr& odom);

  ros::NodeHandle node_;
  ros::Timer timer_;
  ros::Publisher pose_pub_;
  ros::Publisher odom_pub_;
  ros::Subscriber odom_sub_, imu_sub_, vo_sub_,gps_sub_;
  ros::ServiceServer state_srv_;

  // ekf filter
  OdomEstimation my_filter_;

  // estimated robot pose message to send
  geometry_msgs::PoseWithCovarianceStamped  output_;
  geometry_msgs::TwistWithCovariance wheelTwist_;
  nav_msgs::Odometry odom_output_;
  // robot state
  tf::TransformListener    robot_state_;
  tf::TransformBroadcaster odom_broadcaster_;

  // vectors
  tf::Transform odom_meas_, imu_meas_, vo_meas_, gps_meas_;
  tf::Transform base_vo_init_;
  tf::Transform base_gps_init_;
  tf::StampedTransform camera_base_;
  ros::Time odom_time_, imu_time_, vo_time_, gps_time_;
  ros::Time imu_time_prev_;
  ros::Time odom_stamp_, imu_stamp_, vo_stamp_, gps_stamp_, filter_stamp_;
  ros::Time odom_init_stamp_, imu_init_stamp_, vo_init_stamp_, gps_init_stamp_;
  bool publish_transform_;
  bool odom_active_, imu_active_, vo_active_, gps_active_;
  bool odom_used_, imu_used_, vo_used_, gps_used_;
  bool odom_initializing_, imu_initializing_, vo_initializing_, gps_initializing_;
  double timeout_;
  MatrixWrapper::SymmetricMatrix odom_covariance_, imu_covariance_, vo_covariance_, gps_covariance_;
  bool debug_, self_diagnose_;
  std::string output_frame_, base_footprint_frame_, tf_prefix_;

  // Wheel-odometry validation
  bool   wheel_odometry_validation_enabled; ///< True: node will validate wheel odometry using imu.angular_velocity and linear_acceleration
  bool   wheel_odometry_validation_debug;   ///< True: enable debug topics, messages, etc
  double wheel_odometry_reliability;        ///< 1.0: fully reliable, 0.0: unreliable
  double wheel_odometry_reliability_treshold; ///< Below this reliability value, wheel odom won't be used
  std_msgs::Float32 wheel_odometry_reliability_msg;
  ros::Publisher    wheel_odometry_reliability_pub;
  geometry_msgs::Vector3 imu_angular_velocity;
  geometry_msgs::Vector3 imu_linear_acceleration;
  tf::Vector3 imu_angular_velocity_base;
  tf::Vector3 imu_angular_velocity_base_previous;
  tf::Vector3 imu_angular_acceleration_base;
  tf::Vector3 imu_linear_velocity_base;
  tf::Vector3 imu_linear_acceleration_base;
  tf::Vector3 imu_linear_acceleration_base_previous;
  // Publishers for debugging
  ros::Publisher debug_twist_imu_pub;
  ros::Publisher debug_twist_odom_pub;
  geometry_msgs::Twist debug_twist_imu;
  geometry_msgs::Twist debug_twist_odom;

  // log files for debugging
  std::ofstream odom_file_, imu_file_, vo_file_, gps_file_, corr_file_, time_file_, extra_file_;

  // counters
  unsigned int odom_callback_counter_, imu_callback_counter_, vo_callback_counter_,gps_callback_counter_, ekf_sent_counter_;

}; // class

}; // namespace

#endif
