# Robotous_FT_RFT60

**Catkin code to interface RFT60-HA01 sensor via catkin workspace to a ROS node.**

## Method 1 ( Via rqt_service_caller GUI )

Before you start your journey of making and building different stacks, it's essential to familiarize yourself with the following tools:

1. **Rosdep:** [Rosdep on ROS Wiki](http://wiki.ros.org/rosdep)
2. **Synaptic**
3. If ROS has been set up from source, remember to source the setup script:
```bash
   source ~/ros_catkin_ws/install_isolated/setup.bash
```

4. Add the sourcing script to your .bashrc to make it permanent:
```bash
   echo 'source ~/ros_catkin_ws/install_isolated/setup.bash' >> ~/.bashrc
```

5. Source the .bashrc file to apply the changes:
```bash
   source ~/.bashrc
```
6. Read the helpdoc for ROS package folder and set parameters
   ![4](https://github.com/ami6643432/Robotous_FT_RFT60/assets/23532442/2a6a2857-46b1-47af-bd86-934eae6d6481)
   
   
7. Start serial communication
```bash
   mick@mick-Predator-PHN16-71:~/catkin_ft$ rosparam set /RFT_COM_PORT /dev/ttyUSB0
   mick@mick-Predator-PHN16-71:~/catkin_ft$ rosparam set /RFT_TORQUE_DEVIDER 1000
   mick@mick-Predator-PHN16-71:~/catkin_ft$ rosrun rft_sensor_serial rft_sensor_serial
```
If there is error even after package got built, then go to the Common Errors section.

8. Starting rqt-service caller:
```bash
mick@mick-Predator-PHN16-71:~/catkin_ft$ source devel/setup.bash 
mick@mick-Predator-PHN16-71:~/catkin_ft$ rosrun rqt_service_caller rqt_service_caller
```

9.  Set streaming parameters
    ![Screenshot from 2024-04-20 11-33-36](https://github.com/ami6643432/Robotous_FT_RFT60/assets/23532442/7980707c-c1c0-497d-802e-81e05abae64c)

10. start steaming (via the software)
    ![Screenshot from 2024-04-20 11-34-00](https://github.com/ami6643432/Robotous_FT_RFT60/assets/23532442/07e5568d-1202-4119-aa32-8b3a5b0df637)


11. echo rostopic:
```bash
   mick@mick-Predator-PHN16-71:~/catkin_ft$ rostopic echo /RFT_Force
```

12. Run the sensor and get data on rostopic.
 ```bash

```

#Important Key Points

*  Building ROS from Source: Ensure ROS is built from the source for proper integration.

*  Sourcing ROS: Always remember to source ROS; it's crucial. Alternatively, add it to ~/.bashrc.

*  Using Rosdep: Utilize rosdep to install all necessary dependencies efficiently.

*  Port Access: Check if your port is locked by another process or if you lack access permissions.

#Common Errors

Example of a common error encountered:

```bash
mick@mick-Predator-PHN16-71:~/catkin_ft$ source devel/setup.bash
mick@mick-Predator-PHN16-71:~/catkin_ft$ rosrun rft_sensor_serial rft_sensor_serial
[ INFO] [1713562305.371650824]: RFT Serial port: /dev/ttyUSB0
[ INFO] [1713562305.372459789]: RFT Serial port baud-rate: 115200
[ INFO] [1713562305.372557372]: Force Divider of RFT sensor: 50.000000
[ INFO] [1713562305.372640026]: Torque Divider of RFT sensor: 1000.000000
[ERROR] [1713562305.372774309]: COM Port Open Error
```

#Resolving Errors

Steps to resolve port access issues:
```bash
mick@mick-Predator-PHN16-71:~/catkin_ft$ sudo chmod 666 /dev/ttyUSB0
mick@mick-Predator-PHN16-71:~/catkin_ft$ sudo usermod -a -G dialout $USER
mick@mick-Predator-PHN16-71:~/catkin_ft$ lsof | grep ttyUSB0
mick@mick-Predator-PHN16-71:~/catkin_ft$ source devel/setup.bash
```

#Successful Configuration

Output after resolving the issue:
```bash
mick@mick-Predator-PHN16-71:~/catkin_ft$ rosrun rft_sensor_serial rft_sensor_serial
[ INFO] [1713562410.453484136]: RFT Serial port: /dev/ttyUSB0
[ INFO] [1713562410.453849304]: RFT Serial port baud-rate: 115200
[ INFO] [1713562410.454102646]: Force Divider of RFT sensor: 50.000000
[ INFO] [1713562410.454469397]: Torque Divider of RFT sensor: 1000.000000
[ INFO] [1713562410.456895889]: RFT Force/Torque Sensor <Serial> is ready!!!!
```


## Method 2 ( Directly calling the rosservice via terminal )
1. Complete till the step 7 of Method 1
   
2. Call rosservice /rft_serial_op_service and pass the params from the image in step 9 into the input parameters.

   To start the communication
   ```bash
   mick@mick-Predator-PHN16-71:~/Robotous_FT_RFT60/catkin_ft$ rosservice call /rft_serial_op_service "{opType: 11, param1: 0, param2: 0, param3: 0}"
   ```

   To automatically set the bias values
   ```bash
   mick@mick-Predator-PHN16-71:~/Robotous_FT_RFT60/catkin_ft$ rosservice call /rft_serial_op_service "{opType: 17, param1: 1, param2: 0, param3: 0}"
   ```

   To stop the service:
   ```bash
   mick@mick-Predator-PHN16-71:~/Robotous_FT_RFT60/catkin_ft$ rosservice call /rft_serial_op_service "{opType: 12, param1: 0, param2: 0, param3: 0}"
   ```
  3. Echo the topic to see the F/T data being published
  4. 
     ```bash
      mick@mick-Predator-PHN16-71:~/Robotous_FT_RFT60/catkin_ft$ rostopic list
      mick@mick-Predator-PHN16-71:~/Robotous_FT_RFT60/catkin_ft$ rostopic echo /RFT_FORCE
     ```

  5. Making a simple bash script to automate this is preferred. A simple one named rft_rosservice.sh is created and the following commands added to it

     ```
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
     ```

   
