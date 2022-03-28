; Auto-generated. Do not edit!


(cl:in-package shuttles-srv)


;//! \htmlinclude shuttle_id-request.msg.html

(cl:defclass <shuttle_id-request> (roslisp-msg-protocol:ros-message)
  ((robot
    :reader robot
    :initarg :robot
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type cl:integer
    :initform 0))
)

(cl:defclass shuttle_id-request (<shuttle_id-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shuttle_id-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shuttle_id-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name shuttles-srv:<shuttle_id-request> is deprecated: use shuttles-srv:shuttle_id-request instead.")))

(cl:ensure-generic-function 'robot-val :lambda-list '(m))
(cl:defmethod robot-val ((m <shuttle_id-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shuttles-srv:robot-val is deprecated.  Use shuttles-srv:robot instead.")
  (robot m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <shuttle_id-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shuttles-srv:position-val is deprecated.  Use shuttles-srv:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shuttle_id-request>) ostream)
  "Serializes a message object of type '<shuttle_id-request>"
  (cl:let* ((signed (cl:slot-value msg 'robot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shuttle_id-request>) istream)
  "Deserializes a message object of type '<shuttle_id-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robot) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shuttle_id-request>)))
  "Returns string type for a service object of type '<shuttle_id-request>"
  "shuttles/shuttle_idRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shuttle_id-request)))
  "Returns string type for a service object of type 'shuttle_id-request"
  "shuttles/shuttle_idRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shuttle_id-request>)))
  "Returns md5sum for a message object of type '<shuttle_id-request>"
  "4ce93559e67d465c0b309aaf61a7e04b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shuttle_id-request)))
  "Returns md5sum for a message object of type 'shuttle_id-request"
  "4ce93559e67d465c0b309aaf61a7e04b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shuttle_id-request>)))
  "Returns full string definition for message of type '<shuttle_id-request>"
  (cl:format cl:nil "int32 robot~%int32 position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shuttle_id-request)))
  "Returns full string definition for message of type 'shuttle_id-request"
  (cl:format cl:nil "int32 robot~%int32 position~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shuttle_id-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shuttle_id-request>))
  "Converts a ROS message object to a list"
  (cl:list 'shuttle_id-request
    (cl:cons ':robot (robot msg))
    (cl:cons ':position (position msg))
))
;//! \htmlinclude shuttle_id-response.msg.html

(cl:defclass <shuttle_id-response> (roslisp-msg-protocol:ros-message)
  ((IdShuttle
    :reader IdShuttle
    :initarg :IdShuttle
    :type cl:integer
    :initform 0))
)

(cl:defclass shuttle_id-response (<shuttle_id-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <shuttle_id-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'shuttle_id-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name shuttles-srv:<shuttle_id-response> is deprecated: use shuttles-srv:shuttle_id-response instead.")))

(cl:ensure-generic-function 'IdShuttle-val :lambda-list '(m))
(cl:defmethod IdShuttle-val ((m <shuttle_id-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader shuttles-srv:IdShuttle-val is deprecated.  Use shuttles-srv:IdShuttle instead.")
  (IdShuttle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <shuttle_id-response>) ostream)
  "Serializes a message object of type '<shuttle_id-response>"
  (cl:let* ((signed (cl:slot-value msg 'IdShuttle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <shuttle_id-response>) istream)
  "Deserializes a message object of type '<shuttle_id-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'IdShuttle) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<shuttle_id-response>)))
  "Returns string type for a service object of type '<shuttle_id-response>"
  "shuttles/shuttle_idResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shuttle_id-response)))
  "Returns string type for a service object of type 'shuttle_id-response"
  "shuttles/shuttle_idResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<shuttle_id-response>)))
  "Returns md5sum for a message object of type '<shuttle_id-response>"
  "4ce93559e67d465c0b309aaf61a7e04b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'shuttle_id-response)))
  "Returns md5sum for a message object of type 'shuttle_id-response"
  "4ce93559e67d465c0b309aaf61a7e04b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<shuttle_id-response>)))
  "Returns full string definition for message of type '<shuttle_id-response>"
  (cl:format cl:nil "int32 IdShuttle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'shuttle_id-response)))
  "Returns full string definition for message of type 'shuttle_id-response"
  (cl:format cl:nil "int32 IdShuttle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <shuttle_id-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <shuttle_id-response>))
  "Converts a ROS message object to a list"
  (cl:list 'shuttle_id-response
    (cl:cons ':IdShuttle (IdShuttle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'shuttle_id)))
  'shuttle_id-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'shuttle_id)))
  'shuttle_id-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'shuttle_id)))
  "Returns string type for a service object of type '<shuttle_id>"
  "shuttles/shuttle_id")