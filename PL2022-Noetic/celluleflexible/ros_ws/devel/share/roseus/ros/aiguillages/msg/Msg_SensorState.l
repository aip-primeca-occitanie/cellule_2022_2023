;; Auto-generated. Do not edit!


(when (boundp 'aiguillages::Msg_SensorState)
  (if (not (find-package "AIGUILLAGES"))
    (make-package "AIGUILLAGES"))
  (shadow 'Msg_SensorState (find-package "AIGUILLAGES")))
(unless (find-package "AIGUILLAGES::MSG_SENSORSTATE")
  (make-package "AIGUILLAGES::MSG_SENSORSTATE"))

(in-package "ROS")
;;//! \htmlinclude Msg_SensorState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass aiguillages::Msg_SensorState
  :super ros::object
  :slots (_header _id _CPI _CP _PS _DG _DD ))

(defmethod aiguillages::Msg_SensorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:CPI __CPI) (let (r) (dotimes (i 9) (push nil r)) r))
    ((:CP __CP) (let (r) (dotimes (i 11) (push nil r)) r))
    ((:PS __PS) (let (r) (dotimes (i 25) (push nil r)) r))
    ((:DG __DG) (let (r) (dotimes (i 13) (push nil r)) r))
    ((:DD __DD) (let (r) (dotimes (i 13) (push nil r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _CPI __CPI)
   (setq _CP __CP)
   (setq _PS __PS)
   (setq _DG __DG)
   (setq _DD __DD)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:CPI
   (&optional __CPI)
   (if __CPI (setq _CPI __CPI)) _CPI)
  (:CP
   (&optional __CP)
   (if __CP (setq _CP __CP)) _CP)
  (:PS
   (&optional __PS)
   (if __PS (setq _PS __PS)) _PS)
  (:DG
   (&optional __DG)
   (if __DG (setq _DG __DG)) _DG)
  (:DD
   (&optional __DD)
   (if __DD (setq _DD __DD)) _DD)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _id
    4
    ;; bool[9] _CPI
    (* 1    9)
    ;; bool[11] _CP
    (* 1    11)
    ;; bool[25] _PS
    (* 1    25)
    ;; bool[13] _DG
    (* 1    13)
    ;; bool[13] _DD
    (* 1    13)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _id
       (write-long _id s)
     ;; bool[9] _CPI
     (dotimes (i 9)
       (if (elt _CPI i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[11] _CP
     (dotimes (i 11)
       (if (elt _CP i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[25] _PS
     (dotimes (i 25)
       (if (elt _PS i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[13] _DG
     (dotimes (i 13)
       (if (elt _DG i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; bool[13] _DD
     (dotimes (i 13)
       (if (elt _DD i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool[9] _CPI
   (dotimes (i (length _CPI))
     (setf (elt _CPI i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; bool[11] _CP
   (dotimes (i (length _CP))
     (setf (elt _CP i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; bool[25] _PS
   (dotimes (i (length _PS))
     (setf (elt _PS i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; bool[13] _DG
   (dotimes (i (length _DG))
     (setf (elt _DG i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;; bool[13] _DD
   (dotimes (i (length _DD))
     (setf (elt _DD i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     )
   ;;
   self)
  )

(setf (get aiguillages::Msg_SensorState :md5sum-) "dd7658e6685c951b5f6486f4677d2be9")
(setf (get aiguillages::Msg_SensorState :datatype-) "aiguillages/Msg_SensorState")
(setf (get aiguillages::Msg_SensorState :definition-)
      "Header header

int32 id
bool[9] CPI
bool[11] CP
bool[25] PS
bool[13] DG
bool[13] DD

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



(provide :aiguillages/Msg_SensorState "dd7658e6685c951b5f6486f4677d2be9")


