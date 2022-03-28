
(cl:in-package :asdf)

(defsystem "schneider-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Control_cellule" :depends-on ("_package_Control_cellule"))
    (:file "_package_Control_cellule" :depends-on ("_package"))
    (:file "Msg_SensorState" :depends-on ("_package_Msg_SensorState"))
    (:file "_package_Msg_SensorState" :depends-on ("_package"))
  ))