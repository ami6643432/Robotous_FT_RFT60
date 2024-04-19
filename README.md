# Robotous_FT_RFT60

**Catkin code to interface RFT60-HA01 sensor via catkin workspace to a ROS node.**

## Learning Tools

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
6. Read the helpdoc for ROS package folder.
7. Start serial communication
```bash
   mick@mick-Predator-PHN16-71:~/catkin_ft$ rosrun rft_sensor_serial rft_sensor_serial
```
If there is error even after package got built, then go to the Common Errors section.

8. Starting rqt-service caller:
```bash
mick@mick-Predator-PHN16-71:~/catkin_ft$ source devel/setup.bash 
mick@mick-Predator-PHN16-71:~/catkin_ft$ rosrun rqt_service_caller rqt_service_caller
   
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

#
