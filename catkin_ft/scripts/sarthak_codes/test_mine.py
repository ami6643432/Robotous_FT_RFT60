#!/usr/bin/env python
import os
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

ADDR_TORQUE_ENABLE      = 64

ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

ADDR_GOAL_CURRENT      = 102
ADDR_PRESENT_CURRENT   = 126

ADDR_GOAL_VELOCITY      = 102
ADDR_PRESENT_VELOCITY   = 128

ADDR_DRIVE_MODE   = 11

PROTOCOL_VERSION            = 2.0

DXL_ID                      = 1                 
BAUDRATE                    = 57600            
DEVICENAME                  = '/dev/ttyUSB0' 



rospy.init_node('test_mine')
print('hello')
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

try:
	portHandler.openPort()
	print("Succeeded to open the port")
except:

	print("Failed to open the port")
	print("Press any key to terminate...")
	getch()
	quit()

try:
	portHandler.setBaudRate(BAUDRATE)
	print("Succeeded to change the baudrate")
except:
	print("Failed to change the baudrate")
	print("Press any key to terminate...")
	getch()
	quit()

dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 1)
print("%s" % packetHandler.getRxPacketError(dxl_error))

if dxl_comm_result != COMM_SUCCESS:
	print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
	print("Press any key to terminate...")
	getch()
	quit()
elif dxl_error != 0:
	print("%s" % packetHandler.getRxPacketError(dxl_error))
	print("Press any key to terminate...")
	getch()
	quit()
else:
	print("DYNAMIXEL has been successfully connected")


dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 999)
print(dxl_comm_result, dxl_error)
print("%s" % packetHandler.getRxPacketError(dxl_error))
dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT)
print(dxl_present_position, dxl_comm_result, dxl_error)
print("%s" % packetHandler.getRxPacketError(dxl_error))


rospy.spin()