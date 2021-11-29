cmake_minimum_required(VERSION 2.8.3)
project(tortuga_odometry)

add_compile_options(-std=c++11)

# bfl (Bayesian Filtering Library) is a third party package that uses pkg-config
find_package(PkgConfig)
pkg_check_modules(BFL REQUIRED orocos-bfl)

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  image_transport
)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(ekf_estimator
                       src/odom_estimation.cpp
                       src/nonlinearanalyticconditionalgaussianodo.cpp
                       src/odom_estimation_node.cpp)
target_link_libraries(ekf_estimator
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${BFL_LIBRARIES}
    )


#############
## Install ##
#############

install(TARGETS ekf_estimator
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/tortuga_odometry/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY params
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
