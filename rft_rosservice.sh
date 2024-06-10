#!/bin/bash

# roscore in the background
roscore &

# Source the FT sensor workspace
source ~/Robotous_FT_RFT60/catkin_ft/devel/setup.bash


# Set the necessary parameters
rosparam set /RFT_COM_PORT /dev/ttyUSB0
rosparam set /RFT_TORQUE_DEVIDER 1000

# Run the rft_sensor_serial node in the background
rosrun rft_sensor_serial rft_sensor_serial &

# Capture the process ID of the background job
NODE_PID=$!

# Function to stop the service and clean up
cleanup() {
    rosservice call /rft_serial_op_service "{opType: 12, param1: 0, param2: 0, param3: 0}"
    kill $NODE_PID
}

# Set trap to catch termination signals and run cleanup
trap cleanup SIGINT SIGTERM

# Wait for the node to initialize
sleep 2

# Set bias values
rosservice call /rft_serial_op_service "{opType: 17, param1: 1, param2: 0, param3: 0}"


# Start communication
rosservice call /rft_serial_op_service "{opType: 11, param1: 0, param2: 0, param3: 0}"


# Echo the topic
rostopic echo /RFT_FORCE

# Wait indefinitely until the script is terminated
wait $NODE_PID