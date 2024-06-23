execute_process(COMMAND "/my_ros_data/pa_ws/build/fiducials/aruco_detect/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/my_ros_data/pa_ws/build/fiducials/aruco_detect/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
