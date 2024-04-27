
(cl:in-package :asdf)

(defsystem "rft_sensor_serial-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "multiplier" :depends-on ("_package_multiplier"))
    (:file "_package_multiplier" :depends-on ("_package"))
    (:file "rft_operation" :depends-on ("_package_rft_operation"))
    (:file "_package_rft_operation" :depends-on ("_package"))
  ))