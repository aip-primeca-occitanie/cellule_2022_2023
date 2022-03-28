; Auto-generated. Do not edit!


(cl:in-package commande_locale-msg)


;//! \htmlinclude DeplacerPieceMsg.msg.html

(cl:defclass <DeplacerPieceMsg> (roslisp-msg-protocol:ros-message)
  ((num_robot
    :reader num_robot
    :initarg :num_robot
    :type cl:integer
    :initform 0)
   (positionA
    :reader positionA
    :initarg :positionA
    :type cl:integer
    :initform 0)
   (positionB
    :reader positionB
    :initarg :positionB
    :type cl:integer
    :initform 0))
)

(cl:defclass DeplacerPieceMsg (<DeplacerPieceMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DeplacerPieceMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DeplacerPieceMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-msg:<DeplacerPieceMsg> is deprecated: use commande_locale-msg:DeplacerPieceMsg instead.")))

(cl:ensure-generic-function 'num_robot-val :lambda-list '(m))
(cl:defmethod num_robot-val ((m <DeplacerPieceMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:num_robot-val is deprecated.  Use commande_locale-msg:num_robot instead.")
  (num_robot m))

(cl:ensure-generic-function 'positionA-val :lambda-list '(m))
(cl:defmethod positionA-val ((m <DeplacerPieceMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:positionA-val is deprecated.  Use commande_locale-msg:positionA instead.")
  (positionA m))

(cl:ensure-generic-function 'positionB-val :lambda-list '(m))
(cl:defmethod positionB-val ((m <DeplacerPieceMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:positionB-val is deprecated.  Use commande_locale-msg:positionB instead.")
  (positionB m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DeplacerPieceMsg>) ostream)
  "Serializes a message object of type '<DeplacerPieceMsg>"
  (cl:let* ((signed (cl:slot-value msg 'num_robot)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'positionA)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'positionB)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DeplacerPieceMsg>) istream)
  "Deserializes a message object of type '<DeplacerPieceMsg>"
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
      (cl:setf (cl:slot-value msg 'positionA) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'positionB) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DeplacerPieceMsg>)))
  "Returns string type for a message object of type '<DeplacerPieceMsg>"
  "commande_locale/DeplacerPieceMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DeplacerPieceMsg)))
  "Returns string type for a message object of type 'DeplacerPieceMsg"
  "commande_locale/DeplacerPieceMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DeplacerPieceMsg>)))
  "Returns md5sum for a message object of type '<DeplacerPieceMsg>"
  "3590f7e74b621fec7039b4d9aecf39ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DeplacerPieceMsg)))
  "Returns md5sum for a message object of type 'DeplacerPieceMsg"
  "3590f7e74b621fec7039b4d9aecf39ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DeplacerPieceMsg>)))
  "Returns full string definition for message of type '<DeplacerPieceMsg>"
  (cl:format cl:nil "int32 num_robot~%int32 positionA~%int32 positionB~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DeplacerPieceMsg)))
  "Returns full string definition for message of type 'DeplacerPieceMsg"
  (cl:format cl:nil "int32 num_robot~%int32 positionA~%int32 positionB~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DeplacerPieceMsg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DeplacerPieceMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'DeplacerPieceMsg
    (cl:cons ':num_robot (num_robot msg))
    (cl:cons ':positionA (positionA msg))
    (cl:cons ':positionB (positionB msg))
))
