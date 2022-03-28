;; Auto-generated. Do not edit!


(when (boundp 'robots::Msg_numrobot)
  (if (not (find-package "ROBOTS"))
    (make-package "ROBOTS"))
  (shadow 'Msg_numrobot (find-package "ROBOTS")))
(unless (find-package "ROBOTS::MSG_NUMROBOT")
  (make-package "ROBOTS::MSG_NUMROBOT"))

(in-package "ROS")
;;//! \htmlinclude Msg_numrobot.msg.html


(defclass robots::Msg_numrobot
  :super ros::object
  :slots (_num_robot _data ))

(defmethod robots::Msg_numrobot
  (:init
   (&key
    ((:num_robot __num_robot) 0)
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _num_robot (round __num_robot))
   (setq _data (round __data))
   self)
  (:num_robot
   (&optional __num_robot)
   (if __num_robot (setq _num_robot __num_robot)) _num_robot)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _num_robot
    4
    ;; int32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num_robot
       (write-long _num_robot s)
     ;; int32 _data
       (write-long _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num_robot
     (setq _num_robot (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robots::Msg_numrobot :md5sum-) "47287769d49229e4971e3462ae2d0aac")
(setf (get robots::Msg_numrobot :datatype-) "robots/Msg_numrobot")
(setf (get robots::Msg_numrobot :definition-)
      "int32 num_robot
int32 data

")



(provide :robots/Msg_numrobot "47287769d49229e4971e3462ae2d0aac")


