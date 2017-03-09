cmake_minimum_required(VERSION 2.8.3)
project(bluetooth_pair)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
)

catkin_python_setup()

add_message_files(FILES
    BluetoothDevice.msg
    BluetoothDevices.msg
)
add_service_files(FILES
    SearchDevices.srv
    PairDevice.srv
)
generate_messages()

catkin_package(DEPENDS message_runtime)

catkin_install_python(PROGRAMS scripts/bluetooth_pair
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_python()
  roslint_add_test()
endif()
