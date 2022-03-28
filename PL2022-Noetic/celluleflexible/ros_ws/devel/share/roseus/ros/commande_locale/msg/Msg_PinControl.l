;; Auto-generated. Do not edit!


(when (boundp 'commande_locale::Msg_PinControl)
  (if (not (find-package "COMMANDE_LOCALE"))
    (make-package "COMMANDE_LOCALE"))
  (shadow 'Msg_PinControl (find-package "COMMANDE_LOCALE")))
(unless (find-package "COMMANDE_LOCALE::MSG_PINCONTROL")
  (make-package "COMMANDE_LOCALE::MSG_PINCONTROL"))

(in-package "ROS")
;;//! \htmlinclude Msg_PinControl.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass commande_locale::Msg_PinControl
  :super ros::object
  :slots (_header _PINON _PINOFF ))

(defmethod commande_locale::Msg_PinControl
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:PINON __PINON) (let (r) (dotimes (i 9) (push nil r)) r))
    ((:PINOFF __PINOFF) (let (r) (dotimes (i 9) (push nil r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _PINON __PINON)
   (setq _PINOFF __PINOFF)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:PINON
   (&optional __PINON)
   (if __PINON (setq _PINON __PINON)) _PINON)
  (:PINOFF
   (&optional __PINOFF)
   (if __PINOFF (setq _PINOFF __PINOFF)) _PINOFF)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool[9] _PINON
    (* 1    9)
    ;; bool[9] _PINOFF
    (* 1    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool[9] _PINON
     (dotimes (i 9)
       (if (elt _PINON i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[9] _PINOFF
     (dotimes (i 9)
       (if (elt _PINOFF i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool[9] _PINON
   (dotimes (i (length _PINON))
     (setf (elt _PINON i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; bool[9] _PINOFF
   (dotimes (i (length _PINOFF))
     (setf (elt _PINOFF i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;;
   self)
  )

(setf (get commande_locale::Msg_PinControl :md5sum-) "bdef2ee2c42b42b5f1efd9be2b5cd147")
(setf (get commande_locale::Msg_PinControl :datatype-) "commande_locale/Msg_PinControl")
(setf (get commande_locale::Msg_PinControl :definition-)
      "Header header

bool[9] PINON   
bool[9] PINOFF



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



(provide :commande_locale/Msg_PinControl "bdef2ee2c42b42b5f1efd9be2b5cd147")


