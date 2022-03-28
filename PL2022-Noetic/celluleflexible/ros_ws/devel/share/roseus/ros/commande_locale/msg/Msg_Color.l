;; Auto-generated. Do not edit!


(when (boundp 'commande_locale::Msg_Color)
  (if (not (find-package "COMMANDE_LOCALE"))
    (make-package "COMMANDE_LOCALE"))
  (shadow 'Msg_Color (find-package "COMMANDE_LOCALE")))
(unless (find-package "COMMANDE_LOCALE::MSG_COLOR")
  (make-package "COMMANDE_LOCALE::MSG_COLOR"))

(in-package "ROS")
;;//! \htmlinclude Msg_Color.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass commande_locale::Msg_Color
  :super ros::object
  :slots (_header _Handle _Color ))

(defmethod commande_locale::Msg_Color
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:Handle __Handle) 0)
    ((:Color __Color) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _Handle (round __Handle))
   (setq _Color (round __Color))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:Handle
   (&optional __Handle)
   (if __Handle (setq _Handle __Handle)) _Handle)
  (:Color
   (&optional __Color)
   (if __Color (setq _Color __Color)) _Color)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _Handle
    4
    ;; int32 _Color
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _Handle
       (write-long _Handle s)
     ;; int32 _Color
       (write-long _Color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _Handle
     (setq _Handle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _Color
     (setq _Color (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get commande_locale::Msg_Color :md5sum-) "bc181530e810a3c987e52bb86805fe8b")
(setf (get commande_locale::Msg_Color :datatype-) "commande_locale/Msg_Color")
(setf (get commande_locale::Msg_Color :definition-)
      "Header header

int32 Handle
int32 Color

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



(provide :commande_locale/Msg_Color "bc181530e810a3c987e52bb86805fe8b")


