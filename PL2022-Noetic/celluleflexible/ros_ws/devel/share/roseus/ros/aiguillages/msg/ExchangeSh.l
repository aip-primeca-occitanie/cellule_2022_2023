;; Auto-generated. Do not edit!


(when (boundp 'aiguillages::ExchangeSh)
  (if (not (find-package "AIGUILLAGES"))
    (make-package "AIGUILLAGES"))
  (shadow 'ExchangeSh (find-package "AIGUILLAGES")))
(unless (find-package "AIGUILLAGES::EXCHANGESH")
  (make-package "AIGUILLAGES::EXCHANGESH"))

(in-package "ROS")
;;//! \htmlinclude ExchangeSh.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass aiguillages::ExchangeSh
  :super ros::object
  :slots (_header _handle ))

(defmethod aiguillages::ExchangeSh
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:handle __handle) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _handle (round __handle))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:handle
   (&optional __handle)
   (if __handle (setq _handle __handle)) _handle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _handle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _handle
       (write-long _handle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _handle
     (setq _handle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get aiguillages::ExchangeSh :md5sum-) "1e674e2a2bd56787e34ea71041f1bf6d")
(setf (get aiguillages::ExchangeSh :datatype-) "aiguillages/ExchangeSh")
(setf (get aiguillages::ExchangeSh :definition-)
      "Header header
int32 handle

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :aiguillages/ExchangeSh "1e674e2a2bd56787e34ea71041f1bf6d")


