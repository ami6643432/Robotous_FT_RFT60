#!/usr/bin/env python
import os
import rospy
import dynamixel_sdk
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from std_msgs.msg import Int32,Float64MultiArray
import numpy as np

from std_msgs.msg import Float32,Float64MultiArray,Int32
import time

class dynamixtest:

	def shutdown(self):
		print('shutdown')
		self.write_to_motor(0)
		#dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
		#print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		#self.write_to_motor(0)

	def __init__(self,id):
		rospy.init_node('test_mine2')
		rospy.on_shutdown(self.shutdown)

		self.M1_POS = 0
		self.M1_VEL = 0

		self.ADDR_TORQUE_ENABLE      = 64

		self.ADDR_GOAL_POSITION      = 116
		self.ADDR_PRESENT_POSITION   = 132
		self.GOAL_POSITION      	 = 0
		self.PRESENT_POSITION   	 = 0
		self.GOAL_POSITION_ANGLE = 0

		self.ADDR_GOAL_CURRENT       = 102
		self.ADDR_PRESENT_CURRENT    = 126
		self.GOAL_CURRENT       	 = 0 #-1193 to +1193
		self.PRESENT_CURRENT    	 = 0

		self.ADDR_GOAL_VELOCITY      = 104
		self.ADDR_PRESENT_VELOCITY   = 128
		self.GOAL_VELOCITY       	 = 0
		self.PRESENT_VELOCITY    	 = 0

		self.ADDR_OPERATING_MODE     = 11
		self.OPERATING_MODE          = 0 #Current Mode 
		#self.OPERATING_MODE          = 3 #Position mode  

		self.PROTOCOL_VERSION        = 2.0

		self.DXL_ID                  = id                
		self.BAUDRATE                = 57600            
		self.DEVICENAME              = '/dev/ttyUSB0' 
		self.TORQUE_ENABLE           = 1

		if self.DXL_ID == 1:	
			self.kp = 0.05
			self.kd = 0.1
			self.GOAL_POSITION_ANGLE=0
		if self.DXL_ID == 2:	
			self.kp = 0.05
			self.kd = 0.25

		self.error = 0
		self.previous_error=0

		self.pub = rospy.Publisher('error'+str(self.DXL_ID), Float32, queue_size=10)
		self.pub2 = rospy.Publisher('zero'+str(self.DXL_ID), Int32, queue_size=10)

		self.sub2 = rospy.Subscriber("dynamixel"+str(self.DXL_ID)+"_control", Float64MultiArray, self.data_set1)

		self.start_val = 0
		self.init_motor_connection()
		self.tick_ratio = (1024/90)


	def data_set1(self,data):
		self.M1_POS = data.data[0]
		self.M1_VEL = data.data[1]
		self.GOAL_POSITION_ANGLE = self.M1_POS
		self.GOAL_VELOCITY = self.M1_VEL

	def init_motor_connection(self):
		self.portHandler = PortHandler(self.DEVICENAME)
		self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

		try:
			self.portHandler.openPort()
			print("Succeeded to open the port")
		except:
			print("Failed to open the port")
			quit()

		try:
			self.portHandler.setBaudRate(self.BAUDRATE)
			print("Succeeded to change the baudrate")
		except:
			print("Failed to change the baudrate")
			quit()

		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
		print("%s" % self.packetHandler.getRxPacketError(dxl_comm_result))
		# print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		# dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
		# print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		# dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
		# print("%s" % self.packetHandler.getRxPacketError(dxl_error))

		if dxl_comm_result != COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
			quit()
		elif dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(dxl_error))
			quit()
		else:
			print("DYNAMIXEL has been successfully connected")

	def get_motor_data(self):
		self.PRESENT_POSITION, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
		self.PRESENT_VELOCITY, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
		#self.PRESENT_CURRENT, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_CURRENT)

		if self.PRESENT_VELOCITY > 0x7fffffff:
			self.PRESENT_VELOCITY = self.PRESENT_VELOCITY - 4294967296

		if self.PRESENT_POSITION > 0x7fffffff:
			self.PRESENT_POSITION = self.PRESENT_POSITION - 4294967296
		
		#self.PRESENT_POSITION = self.map(self.PRESENT_POSITION,0,65536,-32768,32768)
		#print(self.PRESENT_VELOCITY,self.PRESENT_POSITION)

	def write_to_motor(self,current):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, current)

	def pid(self):
		self.GOAL_POSITION = self.GOAL_POSITION_ANGLE * self.tick_ratio
		self.error = self.GOAL_POSITION - self.PRESENT_POSITION
		self.pub.publish((self.PRESENT_VELOCITY - self.GOAL_VELOCITY)/self.tick_ratio)
		self.pub2.publish(0)
		p = self.error * self.kp
		d = (self.error - self.previous_error) * self.kd
		#d = (self.PRESENT_VELOCITY-self.GOAL_VELOCITY) * self.kd

		pd = int(p+d)

		if pd > 1193:
			pd = 1193
		if pd < -1193:
			pd = -1193 

		print(self.PRESENT_POSITION/self.tick_ratio)

		self.write_to_motor(pd)
		self.previous_error = self.error

class Trajectory:

	def __init__(self):
		self.sub1 = rospy.Subscriber("dynamixel1_cont", Float32, self.cb_mot1)
		self.sub2 = rospy.Subscriber("dynamixel2_cont", Float32, self.cb_mot2)

		self.pub1 = rospy.Publisher('dynamixel1_control', Float64MultiArray, queue_size=10)
		self.pub2 = rospy.Publisher('dynamixel2_control', Float64MultiArray, queue_size=10)

		self.mot1_target = 0
		self.mot1_current = 0
		self.mot1_step = 1

		self.mot2_target = 0
		self.mot2_current = 0
		self.mot2_step = 1

		self.t = 1
		self.k = 10

		#with k 0.5 max speed is achieved

	def cb_mot1(self,data):
		self.mot1_target = data.data

	def cb_mot2(self,data):
		self.mot2_target = data.data
	
	def publish_trajectory(self):
		if (self.mot1_target - self.mot1_current) > 0: 
			self.mot1_current = self.mot1_current + (self.mot1_step/self.t)

		elif (self.mot1_target - self.mot1_current) < 0:
			self.mot1_current = self.mot1_current - (self.mot1_step/self.t)

		if (self.mot2_target - self.mot2_current) > 0: 
			self.mot2_current = self.mot2_current + (self.mot2_step/self.t)

		elif (self.mot2_target - self.mot2_current) < 0:
			self.mot2_current = self.mot2_current - (self.mot2_step/self.t)

		pubs1 = Float64MultiArray()
		pubs1.data = np.array([self.mot1_current,self.t])

		pubs2 = Float64MultiArray()
		pubs2.data = np.array([self.mot2_current,self.t])

		self.pub1.publish(pubs1)
		self.pub2.publish(pubs2)
		#print(self.mot1_step/self.t)

	def set_time(self,t):
		self.t = self.k*t


if __name__ == '__main__':
	obj = dynamixtest(1)
	#obj1 = dynamixtest(2)
	traj = Trajectory()
	t_prev = time.time()
	r = rospy.Rate(60)

	while not rospy.is_shutdown():
		t=time.time()
		# obj.pid()
		# traj.set_time(t-t_prev)
		# traj.publish_trajectory()
		# #obj1.pid()
		# obj.get_motor_data()
		# #obj1.get_motor_data()
		# r.sleep()
		# t_prev = t
	obj.shutdown()
	#obj1.shutdown()