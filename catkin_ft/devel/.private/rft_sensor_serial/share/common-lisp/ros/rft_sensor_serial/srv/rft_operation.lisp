; Auto-generated. Do not edit!


(cl:in-package rft_sensor_serial-srv)


;//! \htmlinclude rft_operation-request.msg.html

(cl:defclass <rft_operation-request> (roslisp-msg-protocol:ros-message)
  ((opType
    :reader opType
    :initarg :opType
    :type cl:fixnum
    :initform 0)
   (param1
    :reader param1
    :initarg :param1
    :type cl:fixnum
    :initform 0)
   (param2
    :reader param2
    :initarg :param2
    :type cl:fixnum
    :initform 0)
   (param3
    :reader param3
    :initarg :param3
    :type cl:fixnum
    :initform 0))
)

(cl:defclass rft_operation-request (<rft_operation-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rft_operation-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rft_operation-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rft_sensor_serial-srv:<rft_operation-request> is deprecated: use rft_sensor_serial-srv:rft_operation-request instead.")))

(cl:ensure-generic-function 'opType-val :lambda-list '(m))
(cl:defmethod opType-val ((m <rft_operation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rft_sensor_serial-srv:opType-val is deprecated.  Use rft_sensor_serial-srv:opType instead.")
  (opType m))

(cl:ensure-generic-function 'param1-val :lambda-list '(m))
(cl:defmethod param1-val ((m <rft_operation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rft_sensor_serial-srv:param1-val is deprecated.  Use rft_sensor_serial-srv:param1 instead.")
  (param1 m))

(cl:ensure-generic-function 'param2-val :lambda-list '(m))
(cl:defmethod param2-val ((m <rft_operation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rft_sensor_serial-srv:param2-val is deprecated.  Use rft_sensor_serial-srv:param2 instead.")
  (param2 m))

(cl:ensure-generic-function 'param3-val :lambda-list '(m))
(cl:defmethod param3-val ((m <rft_operation-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rft_sensor_serial-srv:param3-val is deprecated.  Use rft_sensor_serial-srv:param3 instead.")
  (param3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rft_operation-request>) ostream)
  "Serializes a message object of type '<rft_operation-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'opType)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'param1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'param2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'param3)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rft_operation-request>) istream)
  "Deserializes a message object of type '<rft_operation-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'opType)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'param1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'param2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'param3)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rft_operation-request>)))
  "Returns string type for a service object of type '<rft_operation-request>"
  "rft_sensor_serial/rft_operationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rft_operation-request)))
  "Returns string type for a service object of type 'rft_operation-request"
  "rft_sensor_serial/rft_operationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rft_operation-request>)))
  "Returns md5sum for a message object of type '<rft_operation-request>"
  "5b818d3a3d5cb051d3d33d376492250b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rft_operation-request)))
  "Returns md5sum for a message object of type 'rft_operation-request"
  "5b818d3a3d5cb051d3d33d376492250b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rft_operation-request>)))
  "Returns full string definition for message of type '<rft_operation-request>"
  (cl:format cl:nil "uint8 opType~%uint8 param1~%uint8 param2~%uint8 param3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rft_operation-request)))
  "Returns full string definition for message of type 'rft_operation-request"
  (cl:format cl:nil "uint8 opType~%uint8 param1~%uint8 param2~%uint8 param3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rft_operation-request>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rft_operation-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rft_operation-request
    (cl:cons ':opType (opType msg))
    (cl:cons ':param1 (param1 msg))
    (cl:cons ':param2 (param2 msg))
    (cl:cons ':param3 (param3 msg))
))
;//! \htmlinclude rft_operation-response.msg.html

(cl:defclass <rft_operation-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass rft_operation-response (<rft_operation-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rft_operation-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rft_operation-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rft_sensor_serial-srv:<rft_operation-response> is deprecated: use rft_sensor_serial-srv:rft_operation-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <rft_operation-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rft_sensor_serial-srv:result-val is deprecated.  Use rft_sensor_serial-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rft_operation-response>) ostream)
  "Serializes a message object of type '<rft_operation-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rft_operation-response>) istream)
  "Deserializes a message object of type '<rft_operation-response>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'result)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rft_operation-response>)))
  "Returns string type for a service object of type '<rft_operation-response>"
  "rft_sensor_serial/rft_operationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rft_operation-response)))
  "Returns string type for a service object of type 'rft_operation-response"
  "rft_sensor_serial/rft_operationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rft_operation-response>)))
  "Returns md5sum for a message object of type '<rft_operation-response>"
  "5b818d3a3d5cb051d3d33d376492250b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rft_operation-response)))
  "Returns md5sum for a message object of type 'rft_operation-response"
  "5b818d3a3d5cb051d3d33d376492250b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rft_operation-response>)))
  "Returns full string definition for message of type '<rft_operation-response>"
  (cl:format cl:nil "uint8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rft_operation-response)))
  "Returns full string definition for message of type 'rft_operation-response"
  (cl:format cl:nil "uint8 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rft_operation-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rft_operation-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rft_operation-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rft_operation)))
  'rft_operation-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rft_operation)))
  'rft_operation-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rft_operation)))
  "Returns string type for a service object of type '<rft_operation>"
  "rft_sensor_serial/rft_operation")