;; Auto-generated. Do not edit!


(when (boundp 'shuttles::msgShuttleChange)
  (if (not (find-package "SHUTTLES"))
    (make-package "SHUTTLES"))
  (shadow 'msgShuttleChange (find-package "SHUTTLES")))
(unless (find-package "SHUTTLES::MSGSHUTTLECHANGE")
  (make-package "SHUTTLES::MSGSHUTTLECHANGE"))

(in-package "ROS")
;;//! \htmlinclude msgShuttleChange.msg.html


(defclass shuttles::msgShuttleChange
  :super ros::object
  :slots (_handle _argument ))

(defmethod shuttles::msgShuttleChange
  (:init
   (&key
    ((:handle __handle) 0)
    ((:argument __argument) 0)
    )
   (send-super :init)
   (setq _handle (round __handle))
   (setq _argument (round __argument))
   self)
  (:handle
   (&optional __handle)
   (if __handle (setq _handle __handle)) _handle)
  (:argument
   (&optional __argument)
   (if __argument (setq _argument __argument)) _argument)
  (:serialization-length
   ()
   (+
    ;; int32 _handle
    4
    ;; int32 _argument
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _handle
       (write-long _handle s)
     ;; int32 _argument
       (write-long _argument s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _handle
     (setq _handle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _argument
     (setq _argument (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get shuttles::msgShuttleChange :md5sum-) "02c8ec5acb6ca55401e7f0fabb5b8e17")
(setf (get shuttles::msgShuttleChange :datatype-) "shuttles/msgShuttleChange")
(setf (get shuttles::msgShuttleChange :definition-)
      "# argument = destination or product

int32 handle
int32 argument 

")



(provide :shuttles/msgShuttleChange "02c8ec5acb6ca55401e7f0fabb5b8e17")


