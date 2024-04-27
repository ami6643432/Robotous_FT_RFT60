execute_process(COMMAND "/home/mick/Robotous_FT_RFT60/catkin_ft/build/dynamixel_sdk/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/mick/Robotous_FT_RFT60/catkin_ft/build/dynamixel_sdk/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
