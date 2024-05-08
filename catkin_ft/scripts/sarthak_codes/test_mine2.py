#!/usr/bin/env python
import os
import rospy
import dynamixel_sdk
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
from std_msgs.msg import Int32
import numpy as np

class dynamixtest:

	def shutdown(self):
		print('shutdown')
		dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
		print("%s" % self.packetHandler.getRxPacketError(dxl_error))
		self.write_to_motor(0)

	def callback(self,data):
		self.GOAL_POSITION_ANGLE = data.data

	def __init__(self):
		rospy.init_node('test_mine2')
		rospy.on_shutdown(self.shutdown)

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

		self.DXL_ID                  = 1                 
		self.BAUDRATE                = 57600            
		self.DEVICENAME              = '/dev/ttyUSB0' 
		self.TORQUE_ENABLE           = 1

		self.kp = 0.04
		self.kd = 0.1

		self.error = 0
		self.previous_error=0

		self.pub = rospy.Publisher('error', Int32, queue_size=10)
		self.pub2 = rospy.Publisher('zero', Int32, queue_size=10)
		self.sub = rospy.Subscriber("dynamixel_angle_goal", Int32, self.callback)

		self.start_val = 0
		self.init_motor_connection()

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

	def map(self,x, in_min, in_max, out_min, out_max):
		return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
		#return x - 32768
		#return x

	def get_motor_data(self):
		self.PRESENT_POSITION, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
		self.PRESENT_VELOCITY, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
		#self.PRESENT_CURRENT, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_CURRENT)
		
		#self.PRESENT_POSITION = self.map(self.PRESENT_POSITION,0,65536,-32768,32768)
		#print(self.PRESENT_POSITION)

	def write_to_motor(self,current):
		dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_CURRENT, current)

	def pid(self):
		self.GOAL_POSITION = self.GOAL_POSITION_ANGLE * (1024/90) + 1024
		self.error = self.GOAL_POSITION - self.PRESENT_POSITION
		self.pub.publish(self.error)
		self.pub2.publish(0)
		p = self.error * self.kp
		#d = (self.error - self.previous_error) * self.kd
		d = (self.PRESENT_VELOCITY-self.GOAL_VELOCITY) * self.kd

		pd = int(p+d)

		if pd > 1193:
			pd = 1193
		if pd < -1193:
			pd = -1193 

		#print(self.PRESENT_POSITION,self.error)

		self.write_to_motor(pd)
		self.previous_error = self.error


if __name__ == '__main__':
	obj = dynamixtest()
	r = rospy.Rate(50)
	while not rospy.is_shutdown():
		obj.pid()
		obj.get_motor_data()
		r.sleep()
	obj.shutdown()