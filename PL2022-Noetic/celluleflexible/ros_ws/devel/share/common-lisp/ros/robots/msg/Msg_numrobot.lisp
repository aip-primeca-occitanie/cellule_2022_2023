; Auto-generated. Do not edit!


(cl:in-package robots-msg)


;//! \htmlinclude Msg_numrobot.msg.html

(cl:defclass <Msg_numrobot> (roslisp-msg-protocol:ros-message)
  ((num_robot
    :reader num_robot
    :initarg :num_robot
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass Msg_numrobot (<Msg_numrobot>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Msg_numrobot>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Msg_numrobot)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robots-msg:<Msg_numrobot> is deprecated: use robots-msg:Msg_numrobot instead.")))

(cl:ensure-generic-function 'num_robot-val :lambda-list '(m))
(cl:defmethod num_robot-val ((m <Msg_numrobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robots-msg:num_robot-val is deprecated.  Use robots-msg:num_robot instead.")
  (num_robot m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Msg_numrobot>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robots-msg:data-val is deprecated.  Use robots-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Msg_numrobot>) ostream)
  "Serializes a message object of type '<Msg_numrobot>"
  (cl:let* ((signed (cl:slot-value msg 'num_robot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Msg_numrobot>) istream)
  "Deserializes a message object of type '<Msg_numrobot>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_robot) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Msg_numrobot>)))
  "Returns string type for a message object of type '<Msg_numrobot>"
  "robots/Msg_numrobot")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Msg_numrobot)))
  "Returns string type for a message object of type 'Msg_numrobot"
  "robots/Msg_numrobot")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Msg_numrobot>)))
  "Returns md5sum for a message object of type '<Msg_numrobot>"
  "47287769d49229e4971e3462ae2d0aac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Msg_numrobot)))
  "Returns md5sum for a message object of type 'Msg_numrobot"
  "47287769d49229e4971e3462ae2d0aac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Msg_numrobot>)))
  "Returns full string definition for message of type '<Msg_numrobot>"
  (cl:format cl:nil "int32 num_robot~%int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Msg_numrobot)))
  "Returns full string definition for message of type 'Msg_numrobot"
  (cl:format cl:nil "int32 num_robot~%int32 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Msg_numrobot>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Msg_numrobot>))
  "Converts a ROS message object to a list"
  (cl:list 'Msg_numrobot
    (cl:cons ':num_robot (num_robot msg))
    (cl:cons ':data (data msg))
))
