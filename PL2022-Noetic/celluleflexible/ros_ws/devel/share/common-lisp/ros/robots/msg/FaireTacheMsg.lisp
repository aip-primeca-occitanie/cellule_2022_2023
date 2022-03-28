; Auto-generated. Do not edit!


(cl:in-package robots-msg)


;//! \htmlinclude FaireTacheMsg.msg.html

(cl:defclass <FaireTacheMsg> (roslisp-msg-protocol:ros-message)
  ((num_robot
    :reader num_robot
    :initarg :num_robot
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type cl:integer
    :initform 0)
   (duree
    :reader duree
    :initarg :duree
    :type cl:integer
    :initform 0))
)

(cl:defclass FaireTacheMsg (<FaireTacheMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FaireTacheMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FaireTacheMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robots-msg:<FaireTacheMsg> is deprecated: use robots-msg:FaireTacheMsg instead.")))

(cl:ensure-generic-function 'num_robot-val :lambda-list '(m))
(cl:defmethod num_robot-val ((m <FaireTacheMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robots-msg:num_robot-val is deprecated.  Use robots-msg:num_robot instead.")
  (num_robot m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <FaireTacheMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robots-msg:position-val is deprecated.  Use robots-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'duree-val :lambda-list '(m))
(cl:defmethod duree-val ((m <FaireTacheMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robots-msg:duree-val is deprecated.  Use robots-msg:duree instead.")
  (duree m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FaireTacheMsg>) ostream)
  "Serializes a message object of type '<FaireTacheMsg>"
  (cl:let* ((signed (cl:slot-value msg 'num_robot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'position)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'duree)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FaireTacheMsg>) istream)
  "Deserializes a message object of type '<FaireTacheMsg>"
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
      (cl:setf (cl:slot-value msg 'position) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'duree) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FaireTacheMsg>)))
  "Returns string type for a message object of type '<FaireTacheMsg>"
  "robots/FaireTacheMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FaireTacheMsg)))
  "Returns string type for a message object of type 'FaireTacheMsg"
  "robots/FaireTacheMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FaireTacheMsg>)))
  "Returns md5sum for a message object of type '<FaireTacheMsg>"
  "6d9a6fbf544e79eb45c59d3957770241")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FaireTacheMsg)))
  "Returns md5sum for a message object of type 'FaireTacheMsg"
  "6d9a6fbf544e79eb45c59d3957770241")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FaireTacheMsg>)))
  "Returns full string definition for message of type '<FaireTacheMsg>"
  (cl:format cl:nil "int32 num_robot~%int32 position~%int32 duree~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FaireTacheMsg)))
  "Returns full string definition for message of type 'FaireTacheMsg"
  (cl:format cl:nil "int32 num_robot~%int32 position~%int32 duree~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FaireTacheMsg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FaireTacheMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'FaireTacheMsg
    (cl:cons ':num_robot (num_robot msg))
    (cl:cons ':position (position msg))
    (cl:cons ':duree (duree msg))
))
