;; Auto-generated. Do not edit!


(when (boundp 'robots::MoveRobot)
  (if (not (find-package "ROBOTS"))
    (make-package "ROBOTS"))
  (shadow 'MoveRobot (find-package "ROBOTS")))
(unless (find-package "ROBOTS::MOVEROBOT")
  (make-package "ROBOTS::MOVEROBOT"))

(in-package "ROS")
;;//! \htmlinclude MoveRobot.msg.html


(defclass robots::MoveRobot
  :super ros::object
  :slots (_position _bras _pince _num_robot ))

(defmethod robots::MoveRobot
  (:init
   (&key
    ((:position __position) 0)
    ((:bras __bras) 0)
    ((:pince __pince) 0)
    ((:num_robot __num_robot) 0)
    )
   (send-super :init)
   (setq _position (round __position))
   (setq _bras (round __bras))
   (setq _pince (round __pince))
   (setq _num_robot (round __num_robot))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:bras
   (&optional __bras)
   (if __bras (setq _bras __bras)) _bras)
  (:pince
   (&optional __pince)
   (if __pince (setq _pince __pince)) _pince)
  (:num_robot
   (&optional __num_robot)
   (if __num_robot (setq _num_robot __num_robot)) _num_robot)
  (:serialization-length
   ()
   (+
    ;; int32 _position
    4
    ;; int32 _bras
    4
    ;; int32 _pince
    4
    ;; int32 _num_robot
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _position
       (write-long _position s)
     ;; int32 _bras
       (write-long _bras s)
     ;; int32 _pince
       (write-long _pince s)
     ;; int32 _num_robot
       (write-long _num_robot s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _bras
     (setq _bras (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _pince
     (setq _pince (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_robot
     (setq _num_robot (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robots::MoveRobot :md5sum-) "6a71d2b8fb832f571ecc6a220d76058b")
(setf (get robots::MoveRobot :datatype-) "robots/MoveRobot")
(setf (get robots::MoveRobot :definition-)
      "int32 position
int32 bras
int32 pince
int32 num_robot

")



(provide :robots/MoveRobot "6a71d2b8fb832f571ecc6a220d76058b")


