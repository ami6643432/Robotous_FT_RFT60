#!/usr/bin/env python
import sys
# ROS python API
import rospy

from std_msgs.msg import *
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Accel, Vector3
# import all mavros messages and services
from nav_msgs.msg import *
# from trajectory_msgs.msg import MultiDOFJointTrajectory as Mdjt
# from scipy import linalg as la

import dynamixel_sdk
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from tf import *


import sys
import tty
import termios
import select

import numpy as np
import tf
# import RPi.GPIO as GPIO
import time
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class dynamixtest:


    def shutdown(self):
        print('shutdown')
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def __init__(self,id):

        self.PRESENT_POSITION        = 0

        self.OPERATING_MODE          = 0 #Current control mode
        # self.OPERATING_MODE          = 2 #Position mode

        # Control table address

        self.ADDR_OPERATING_MODE     = 11
        self.ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
        
        self.ADDR_GOAL_POSITION      = 116
        self.ADDR_GOAL_CURRENT       = 102

        self.ADDR_PRESENT_VELOCITY   = 128
        self.ADDR_PRESENT_POSITION   = 132
        self.ADDR_PRESENT_CURRENT    = 126

        
  

        # Protocol version
        self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

        # Default setting
        # DXL_ID                      = 1                 # Dynamixel ID : 1
        self.DXL_ID                  = id                
        print(self.DXL_ID)

        self.BAUDRATE                    = 57600
        # self.BAUDRATE                    = 115200             # Dynamixel default baudrate : 57600
        self.DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
        self.TORQUE_DISABLE              = 0                 # Value for disabling the torque
        self.DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
        # DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

        self.DXL_MAXIMUM_POSITION_VALUE  = 10000*(1024/90)            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)

        self.DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        self.init_motor_connection()


    def init_motor_connection(self):  #do not change 
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            getch()
            quit()

        try:
            self.portHandler.setBaudRate(self.BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            getch()
            quit()


        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("DYNAMIXEL disabled")
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("DYNAMIXEL set to current control mode")
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        print("DYNAMIXEL enabled")

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            getch()
            quit()
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            getch()
            quit()
        else:
            print("DYNAMIXEL has been successfully connected")


    def get_motor_data(self):
        self.PRESENT_POSITION, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        self.PRESENT_VELOCITY, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
        self.PRESENT_CURRENT, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_CURRENT)

        # converting the readings to degrees
        self.PRESENT_POSITION = self.PRESENT_POSITION*(90/1024)

        # Adjusting the readings for negative values
        if self.PRESENT_VELOCITY > 4294967296/2:
            self.PRESENT_VELOCITY = self.PRESENT_VELOCITY - 4294967296

        if self.PRESENT_CURRENT > 65536/2:
            self.PRESENT_CURRENT = self.PRESENT_CURRENT - 65536

        # converting current to mA
        self.PRESENT_CURRENT = self.PRESENT_CURRENT * 2.69

        # converting velocity to degrees/sec
        self.PRESENT_VELOCITY = self.PRESENT_VELOCITY * 0.229 * (360/60)


    def set_goal_pos_callback(self, position):
        print("Set Goal Position of ID %s = %s" % (self.DXL_ID, position*(1024/90)))
        pos_ticks = round(position *(1024/90))
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, pos_ticks )

    def set_goal_curr_callback(self, curr_ticks):
        print("Set Goal Current of ID %s = %s mA" % (self.DXL_ID, curr_ticks))
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, curr_ticks )

def is_data():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])




def main():
   
    print("All Good")

    dynamixel_obj1 = dynamixtest(1)
    time.sleep(0.5)
    print("enabled 1")

    dynamixel_obj2 = dynamixtest(2)
    time.sleep(0.5)
    print("enabled 2")

    k0 = 0.1* np.array([1,1])
    k1 = 0.1* np.array([1,1])
    k2 = 0.1* np.array([1,1])


    # BE CAREFUL OF THE MOTOR IDs 
    # dynamixel_obj1.set_goal_curr_callback(50)
    # print("enabled 2")

    # time.sleep(2)
    # dynamixel_obj1.get_motor_data()
    # continuously get motor data while q not pressed

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        while True:
            if is_data():
                c = sys.stdin.read(1)
                if c == '\x1b':  # x1b is ESC
                    dynamixel_obj1.shutdown()
                    dynamixel_obj2.shutdown()
                    break
            
            # Actual code

            #home position alighned with gravity
            pos1h = -90
            pos2h = -180

            #storing values into variables

            posd  = 45
            posd2 = 90

            #Dynamixel get motor data
            dynamixel_obj1.get_motor_data() 
            dynamixel_obj2.get_motor_data() 

            pos = dynamixel_obj1.PRESENT_POSITION + pos1h # degrees
            cur = dynamixel_obj1.PRESENT_CURRENT  # mA
            vel = dynamixel_obj1.PRESENT_VELOCITY # deg/sec

            pos2 = dynamixel_obj2.PRESENT_POSITION + pos2h # degrees
            cur2 = dynamixel_obj2.PRESENT_CURRENT  # mA
            vel2 = dynamixel_obj2.PRESENT_VELOCITY # deg/sec

            # defining current and desired state vectors
            q = np.array([pos,pos2])
            q_dot = np.array([vel,vel2])

            qd = np.array([posd,posd2])
            qd_dot = np.array([0,0])


            ep = qd - q
            ev = qd_dot - q_dot

            # defining the system parameters

            m1 = .9 #kg
            m2 = .1 #kg

            L1 = .7 #cm
            L2 = .9 #cm


            #Matrices
            theta1, theta2 = pos*np.pi/180, pos2*np.pi/180  # angles
            theta1_dot, theta2_dot = vel*np.pi/180, vel2*np.pi/180  # angular velocities
            g = 9.81  # acceleration due to gravity

            # Define the M(theta) matrix
            M = np.array([
                [m1 * L1**2 + m2 * (L1**2 + 2 * L1 * L2 * np.sin(theta2) + L2**2), m2 * (L1 * L2 * np.sin(theta2) + L2**2)],
                [m2 * (L1 * L2 * np.sin(theta2) + L2**2), m2 * L2**2]
            ])

            # Define the c(theta, theta_dot) matrix
            c_vector = np.array([
                -m2 * L1 * L2 * np.cos(theta2) * (2 * theta1_dot * theta2_dot + theta2_dot**2),
                m2 * L1 * L2 * theta1_dot**2 * np.cos(theta2)
            ])

            # Define the g(theta) matrix
            g_vector = np.array([
                (m1 + m2) * L1 * g * np.sin(theta1) + m2 * L2 * g * np.sin(theta1 + theta2),
                m2 * L2 * g * np.sin(theta1 + theta2)
            ])


            #Controller

            # Tuning parameters
            lam = 1e-2 *np.array([
                [10, 0],
                [0, 10]
            ])

            phip = np.array([
                [8, 0],
                [0, 8]
            ])

            phiv = np.array([
                [1, 0],
                [0, 2]
            ])


            kg = 5 #weight scaling

            kc = 1/100 #current scaling

            
            s = phiv.dot(ev) + phip.dot(ep)

            # k0 = k0 + 

            zeta = np.transpose([np.transpose(ep), np.transpose(ep)])

            g_vector = g_vector * kg

            c_vector = c_vector * kc

            tau_hat = lam.dot(s)

            tau_id = g_vector + c_vector

            # del_tau = 

            # print("g_vector : ", g_vector)
            # print("c_vector : ", c_vector)

            # print("Tau_ID : ", tau_id)
            # print("Tau_Hat : ", tau_hat)

            tau =  1.2*tau_id + tau_hat

            # tau = tau_id


            tau = np.round(tau)

            tau1 = tau[0]

            tau2 = tau[1]

            # print(dynamixel_obj1.DXL_ID ,"Current  "," : ",dynamixel_obj1.PRESENT_CURRENT*2.69, " mA")
            # print(dynamixel_obj1.DXL_ID ,"Position "," : ",round(dynamixel_obj1.PRESENT_POSITION*0.088,3), " degrees")
            # print(dynamixel_obj1.DXL_ID ,"Velocity "," : ",round(dynamixel_obj1.PRESENT_VELOCITY * 0.229 * (360/60),3), " degrees/sec")
            
            print(dynamixel_obj1.DXL_ID ,"Current  "," : ",cur) # 0 ~ 1,193 	2.69 [mA]
            print(dynamixel_obj1.DXL_ID ,"Position "," : ",pos, " degrees")
            print(dynamixel_obj1.DXL_ID ,"Velocity "," : ",vel) #0 ~ 1,023 	0.229 [rev/min]


            print(dynamixel_obj2.DXL_ID ,"Current  "," : ",cur2) # 0 ~ 1,193 	2.69 [mA]
            print(dynamixel_obj2.DXL_ID ,"Position "," : ",pos2, " degrees")
            print(dynamixel_obj2.DXL_ID ,"Velocity "," : ",vel2) #0 ~ 1,023 	0.229 [rev/min]

            # print("ep1 : %s, ev1 : %s" % ( round(ep,2), round(ev)))
            #print gravity torque
            # print("Tau_Gravity : ", g_vector[0])
            print("Tau1 : ", tau1)

            # print("ep2 : %s, ev2 : %s" % (round(ep2), round(ev2)))
            #print gravity torque
            # print("Tau_Gravity2 : ", g_vector[1])
            print("Tau2 : ", tau2)


            dynamixel_obj1.set_goal_curr_callback(int(tau1))
            dynamixel_obj2.set_goal_curr_callback(int(tau2))

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
