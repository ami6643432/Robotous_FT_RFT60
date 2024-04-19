# Robotous_FT_RFT60
Catkin code to interface RFT60-HA01 sensor via catkin workspace to a ros node

One of the most important tools to learn before starting your journey of making and building different stacks is:
1. Rosdep: http://wiki.ros.org/rosdep
2. Synaptic
3. If ROS has been setup from source then never forget to source ~/ros_catkin_ws/install_isolated/setup.bash.
4. It can be directly added to the path (~./bashrc) via the command : echo 'source ~/ros_catkin_ws/install_isolated/setup.bash' >> ~/.bashrc
5. and then simply source ~/.bashrc would source everything added to that file.




IMPORTANT KEYPOINTS OTHER THAN THE SAMPLE CODE AND DOC:

1.  ROS had to be built from the source.
2.  Sourcing ros is extremely important, dont forget. Otherwise add it to ./bashrc.
3.  Use rosdep to install all dependencies to save on time and effort.
4.  Your port might be locked by a different process, or you might not have the access to it.

Error before.
mick@mick-Predator-PHN16-71:~/catkin_ft$ source devel/setup.bash 
mick@mick-Predator-PHN16-71:~/catkin_ft$ rosrun rft_sensor_serial rft_sensor_serial
[ INFO] [1713562305.371650824]: RFT Serial port: /dev/ttyUSB0
[ INFO] [1713562305.372459789]: RFT Serial port baud-rate: 115200
[ INFO] [1713562305.372557372]: Force Divider of RFT sensor: 50.000000
[ INFO] [1713562305.372640026]: Torque Divider of RFT sensor: 1000.000000
[ERROR] [1713562305.372774309]: COM Port Open Error

Process
mick@mick-Predator-PHN16-71:~/catkin_ft$   sudo chmod 666 /dev/ttyUSB0
mick@mick-Predator-PHN16-71:~/catkin_ft$ sudo usermod -a -G dialout $USER
mick@mick-Predator-PHN16-71:~/catkin_ft$ lsof | grep ttyUSB0
mick@mick-Predator-PHN16-71:~/catkin_ft$ source devel/setup.bash 

After
mick@mick-Predator-PHN16-71:~/catkin_ft$ rosrun rft_sensor_serial rft_sensor_serial
[ INFO] [1713562410.453484136]: RFT Serial port: /dev/ttyUSB0
[ INFO] [1713562410.453849304]: RFT Serial port baud-rate: 115200
[ INFO] [1713562410.454102646]: Force Divider of RFT sensor: 50.000000
[ INFO] [1713562410.454469397]: Torque Divider of RFT sensor: 1000.000000
[ INFO] [1713562410.456895889]: RFT Force/Torque Sensor <Serial> is ready!!!!
^Cmick@mick-Predator-PHN16-71:~/catkin_ft$ ^C
