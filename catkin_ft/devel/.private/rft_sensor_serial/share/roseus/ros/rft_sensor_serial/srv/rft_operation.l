;; Auto-generated. Do not edit!


(when (boundp 'rft_sensor_serial::rft_operation)
  (if (not (find-package "RFT_SENSOR_SERIAL"))
    (make-package "RFT_SENSOR_SERIAL"))
  (shadow 'rft_operation (find-package "RFT_SENSOR_SERIAL")))
(unless (find-package "RFT_SENSOR_SERIAL::RFT_OPERATION")
  (make-package "RFT_SENSOR_SERIAL::RFT_OPERATION"))
(unless (find-package "RFT_SENSOR_SERIAL::RFT_OPERATIONREQUEST")
  (make-package "RFT_SENSOR_SERIAL::RFT_OPERATIONREQUEST"))
(unless (find-package "RFT_SENSOR_SERIAL::RFT_OPERATIONRESPONSE")
  (make-package "RFT_SENSOR_SERIAL::RFT_OPERATIONRESPONSE"))

(in-package "ROS")





(defclass rft_sensor_serial::rft_operationRequest
  :super ros::object
  :slots (_opType _param1 _param2 _param3 ))

(defmethod rft_sensor_serial::rft_operationRequest
  (:init
   (&key
    ((:opType __opType) 0)
    ((:param1 __param1) 0)
    ((:param2 __param2) 0)
    ((:param3 __param3) 0)
    )
   (send-super :init)
   (setq _opType (round __opType))
   (setq _param1 (round __param1))
   (setq _param2 (round __param2))
   (setq _param3 (round __param3))
   self)
  (:opType
   (&optional __opType)
   (if __opType (setq _opType __opType)) _opType)
  (:param1
   (&optional __param1)
   (if __param1 (setq _param1 __param1)) _param1)
  (:param2
   (&optional __param2)
   (if __param2 (setq _param2 __param2)) _param2)
  (:param3
   (&optional __param3)
   (if __param3 (setq _param3 __param3)) _param3)
  (:serialization-length
   ()
   (+
    ;; uint8 _opType
    1
    ;; uint8 _param1
    1
    ;; uint8 _param2
    1
    ;; uint8 _param3
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _opType
       (write-byte _opType s)
     ;; uint8 _param1
       (write-byte _param1 s)
     ;; uint8 _param2
       (write-byte _param2 s)
     ;; uint8 _param3
       (write-byte _param3 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _opType
     (setq _opType (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _param1
     (setq _param1 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _param2
     (setq _param2 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _param3
     (setq _param3 (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass rft_sensor_serial::rft_operationResponse
  :super ros::object
  :slots (_result ))

(defmethod rft_sensor_serial::rft_operationResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; uint8 _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _result
       (write-byte _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _result
     (setq _result (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass rft_sensor_serial::rft_operation
  :super ros::object
  :slots ())

(setf (get rft_sensor_serial::rft_operation :md5sum-) "5b818d3a3d5cb051d3d33d376492250b")
(setf (get rft_sensor_serial::rft_operation :datatype-) "rft_sensor_serial/rft_operation")
(setf (get rft_sensor_serial::rft_operation :request) rft_sensor_serial::rft_operationRequest)
(setf (get rft_sensor_serial::rft_operation :response) rft_sensor_serial::rft_operationResponse)

(defmethod rft_sensor_serial::rft_operationRequest
  (:response () (instance rft_sensor_serial::rft_operationResponse :init)))

(setf (get rft_sensor_serial::rft_operationRequest :md5sum-) "5b818d3a3d5cb051d3d33d376492250b")
(setf (get rft_sensor_serial::rft_operationRequest :datatype-) "rft_sensor_serial/rft_operationRequest")
(setf (get rft_sensor_serial::rft_operationRequest :definition-)
      "uint8 opType
uint8 param1
uint8 param2
uint8 param3
---
uint8 result

")

(setf (get rft_sensor_serial::rft_operationResponse :md5sum-) "5b818d3a3d5cb051d3d33d376492250b")
(setf (get rft_sensor_serial::rft_operationResponse :datatype-) "rft_sensor_serial/rft_operationResponse")
(setf (get rft_sensor_serial::rft_operationResponse :definition-)
      "uint8 opType
uint8 param1
uint8 param2
uint8 param3
---
uint8 result

")



(provide :rft_sensor_serial/rft_operation "5b818d3a3d5cb051d3d33d376492250b")


