#!/usr/bin/env python
import os
import rospy
import dynamixel_sdk
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from std_msgs.msg import Int32,Float32
import numpy as np
import time

class dynamixtest:
	def shutdown(self):
		print('shutdown')
		self.write_to_motor(0)

	def __init__(self,id):
		rospy.init_node('dynamixel_test')
		rospy.on_shutdown(self.shutdown)

		self.ADDR_TORQUE_ENABLE      = 64

		self.ADDR_GOAL_POSITION      = 116
		self.ADDR_PRESENT_POSITION   = 132
		self.GOAL_POSITION      	 = 0
		self.PRESENT_POSITION   	 = 0
		self.GOAL_POSITION_ANGLE     = 0

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
		#self.OPERATING_MODE          = 1 #Velocity mode 
		# self.OPERATING_MODE          = 3 #Position mode 
		#self.OPERATING_MODE          = 4 #Extended Position mode

		self.PROTOCOL_VERSION        = 2.0

		self.DXL_ID                  = id                
		self.BAUDRATE                = 57600            
		self.DEVICENAME              = '/dev/ttyUSB0' 
		self.TORQUE_ENABLE           = 1

		self.pub = rospy.Publisher('encoder'+str(self.DXL_ID), Int32, queue_size=10)
		self.sub2 = rospy.Subscriber("dynamixel"+str(self.DXL_ID)+"_control", Float32, self.set_control_data)

		self.start_val = 0
		self.init_motor_connection()
		self.tick_ratio = (1024/90)

	def set_control_data(self,data):
		if self.OPERATING_MODE == 0:
			self.GOAL_CURRENT = data.data
		if self.OPERATING_MODE == 1:
			self.GOAL_VELOCITY = data.data
		if self.OPERATING_MODE == 3:
			self.GOAL_POSITION_ANGLE = data.data
			self.GOAL_POSITION = self.GOAL_POSITION_ANGLE * self.tick_ratio
		if self.OPERATING_MODE == 4:
			self.GOAL_POSITION_ANGLE = data.data
			self.GOAL_POSITION = self.GOAL_POSITION_ANGLE * self.tick_ratio

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
		print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
		print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
		print("%s" % self.packetHandler.getRxPacketError(dxl_error))

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

	def write_to_motor(self,data):
		if self.OPERATING_MODE == 0:
			dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, int(data))
		if self.OPERATING_MODE == 1:
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY, int(data))
		if self.OPERATING_MODE == 3:
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, int(data))
		if self.OPERATING_MODE == 4:
			dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, int(data))

	def control_loop(self):
		if self.OPERATING_MODE == 0:
			self.write_to_motor(self.GOAL_CURRENT)
		if self.OPERATING_MODE == 1:
			self.write_to_motor(self.GOAL_VELOCITY)
		if self.OPERATING_MODE == 3:
			self.write_to_motor(self.GOAL_POSITION)
		if self.OPERATING_MODE == 4:
			self.write_to_motor(self.GOAL_POSITION)
		

if __name__ == '__main__':
	obj = dynamixtest(1)
	# obj1 = dynamixtest(2)
	r = rospy.Rate(60)
	while not rospy.is_shutdown():
		# obj.control_loop()
		#obj1.control_loop()
		# obj.get_motor_data()
		#obj1.get_motor_data()
		r.sleep()
	obj.shutdown()
	#obj1.shutdown()		