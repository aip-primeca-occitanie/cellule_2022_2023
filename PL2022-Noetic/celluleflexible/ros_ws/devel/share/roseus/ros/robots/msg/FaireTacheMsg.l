;; Auto-generated. Do not edit!


(when (boundp 'robots::FaireTacheMsg)
  (if (not (find-package "ROBOTS"))
    (make-package "ROBOTS"))
  (shadow 'FaireTacheMsg (find-package "ROBOTS")))
(unless (find-package "ROBOTS::FAIRETACHEMSG")
  (make-package "ROBOTS::FAIRETACHEMSG"))

(in-package "ROS")
;;//! \htmlinclude FaireTacheMsg.msg.html


(defclass robots::FaireTacheMsg
  :super ros::object
  :slots (_num_robot _position _duree ))

(defmethod robots::FaireTacheMsg
  (:init
   (&key
    ((:num_robot __num_robot) 0)
    ((:position __position) 0)
    ((:duree __duree) 0)
    )
   (send-super :init)
   (setq _num_robot (round __num_robot))
   (setq _position (round __position))
   (setq _duree (round __duree))
   self)
  (:num_robot
   (&optional __num_robot)
   (if __num_robot (setq _num_robot __num_robot)) _num_robot)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:duree
   (&optional __duree)
   (if __duree (setq _duree __duree)) _duree)
  (:serialization-length
   ()
   (+
    ;; int32 _num_robot
    4
    ;; int32 _position
    4
    ;; int32 _duree
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num_robot
       (write-long _num_robot s)
     ;; int32 _position
       (write-long _position s)
     ;; int32 _duree
       (write-long _duree s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num_robot
     (setq _num_robot (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _duree
     (setq _duree (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robots::FaireTacheMsg :md5sum-) "6d9a6fbf544e79eb45c59d3957770241")
(setf (get robots::FaireTacheMsg :datatype-) "robots/FaireTacheMsg")
(setf (get robots::FaireTacheMsg :definition-)
      "int32 num_robot
int32 position
int32 duree

")



(provide :robots/FaireTacheMsg "6d9a6fbf544e79eb45c59d3957770241")


