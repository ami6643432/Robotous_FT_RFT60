import rospy
# Import the service message python classes generated from the .srv file
# Make sure 'RFT_Sensor_Serial' matches the name of your ROS package
# and 'RftOperation' is the name of the generated service class
from RFT_Sensor_Serial.srv import RftOperation, RftOperationRequest

# Initialize the ROS node
rospy.init_node('service_client_custom_node')

# Define the service name
service_name = '/rft_serial_op_service'

# Wait for the service to be available
rospy.wait_for_service(service_name)

try:
    # Create a service proxy
    service_client = rospy.ServiceProxy(service_name, RftOperation)

    # Create a request object, which you will send
    request = RftOperationRequest()
    request.opType = 11  # Set the parameter values accordingly
    request.param1 = 0
    request.param2 = 0
    request.param3 = 0

    # Call the service and pass the request object
    response = service_client(request)

    # Print the response
    rospy.loginfo("Service response: %s", response)

except rospy.ServiceException as e:
    rospy.logerr("Service call failed: %s", e)
