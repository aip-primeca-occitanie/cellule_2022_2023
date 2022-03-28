;; Auto-generated. Do not edit!


(when (boundp 'commande_locale::RobotJoints)
  (if (not (find-package "COMMANDE_LOCALE"))
    (make-package "COMMANDE_LOCALE"))
  (shadow 'RobotJoints (find-package "COMMANDE_LOCALE")))
(unless (find-package "COMMANDE_LOCALE::ROBOTJOINTS")
  (make-package "COMMANDE_LOCALE::ROBOTJOINTS"))

(in-package "ROS")
;;//! \htmlinclude RobotJoints.msg.html


(defclass commande_locale::RobotJoints
  :super ros::object
  :slots (_joint1 _joint2 _joint3 _joint4 _joint5 _joint6 _joint7 _num_robot ))

(defmethod commande_locale::RobotJoints
  (:init
   (&key
    ((:joint1 __joint1) 0)
    ((:joint2 __joint2) 0)
    ((:joint3 __joint3) 0)
    ((:joint4 __joint4) 0)
    ((:joint5 __joint5) 0)
    ((:joint6 __joint6) 0)
    ((:joint7 __joint7) 0)
    ((:num_robot __num_robot) 0)
    )
   (send-super :init)
   (setq _joint1 (round __joint1))
   (setq _joint2 (round __joint2))
   (setq _joint3 (round __joint3))
   (setq _joint4 (round __joint4))
   (setq _joint5 (round __joint5))
   (setq _joint6 (round __joint6))
   (setq _joint7 (round __joint7))
   (setq _num_robot (round __num_robot))
   self)
  (:joint1
   (&optional __joint1)
   (if __joint1 (setq _joint1 __joint1)) _joint1)
  (:joint2
   (&optional __joint2)
   (if __joint2 (setq _joint2 __joint2)) _joint2)
  (:joint3
   (&optional __joint3)
   (if __joint3 (setq _joint3 __joint3)) _joint3)
  (:joint4
   (&optional __joint4)
   (if __joint4 (setq _joint4 __joint4)) _joint4)
  (:joint5
   (&optional __joint5)
   (if __joint5 (setq _joint5 __joint5)) _joint5)
  (:joint6
   (&optional __joint6)
   (if __joint6 (setq _joint6 __joint6)) _joint6)
  (:joint7
   (&optional __joint7)
   (if __joint7 (setq _joint7 __joint7)) _joint7)
  (:num_robot
   (&optional __num_robot)
   (if __num_robot (setq _num_robot __num_robot)) _num_robot)
  (:serialization-length
   ()
   (+
    ;; int32 _joint1
    4
    ;; int32 _joint2
    4
    ;; int32 _joint3
    4
    ;; int32 _joint4
    4
    ;; int32 _joint5
    4
    ;; int32 _joint6
    4
    ;; int32 _joint7
    4
    ;; int32 _num_robot
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _joint1
       (write-long _joint1 s)
     ;; int32 _joint2
       (write-long _joint2 s)
     ;; int32 _joint3
       (write-long _joint3 s)
     ;; int32 _joint4
       (write-long _joint4 s)
     ;; int32 _joint5
       (write-long _joint5 s)
     ;; int32 _joint6
       (write-long _joint6 s)
     ;; int32 _joint7
       (write-long _joint7 s)
     ;; int32 _num_robot
       (write-long _num_robot s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _joint1
     (setq _joint1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _joint2
     (setq _joint2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _joint3
     (setq _joint3 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _joint4
     (setq _joint4 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _joint5
     (setq _joint5 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _joint6
     (setq _joint6 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _joint7
     (setq _joint7 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_robot
     (setq _num_robot (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get commande_locale::RobotJoints :md5sum-) "83d02ab5b13f8f1d23146613b3ae26a8")
(setf (get commande_locale::RobotJoints :datatype-) "commande_locale/RobotJoints")
(setf (get commande_locale::RobotJoints :definition-)
      "int32 joint1
int32 joint2
int32 joint3
int32 joint4
int32 joint5
int32 joint6
int32 joint7
int32 num_robot

")



(provide :commande_locale/RobotJoints "83d02ab5b13f8f1d23146613b3ae26a8")


