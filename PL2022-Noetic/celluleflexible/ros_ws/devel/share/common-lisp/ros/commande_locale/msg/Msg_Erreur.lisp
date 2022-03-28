; Auto-generated. Do not edit!


(cl:in-package commande_locale-msg)


;//! \htmlinclude Msg_Erreur.msg.html

(cl:defclass <Msg_Erreur> (roslisp-msg-protocol:ros-message)
  ((code
    :reader code
    :initarg :code
    :type cl:integer
    :initform 0)
   (n_poste
    :reader n_poste
    :initarg :n_poste
    :type cl:integer
    :initform 0))
)

(cl:defclass Msg_Erreur (<Msg_Erreur>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Msg_Erreur>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Msg_Erreur)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-msg:<Msg_Erreur> is deprecated: use commande_locale-msg:Msg_Erreur instead.")))

(cl:ensure-generic-function 'code-val :lambda-list '(m))
(cl:defmethod code-val ((m <Msg_Erreur>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:code-val is deprecated.  Use commande_locale-msg:code instead.")
  (code m))

(cl:ensure-generic-function 'n_poste-val :lambda-list '(m))
(cl:defmethod n_poste-val ((m <Msg_Erreur>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:n_poste-val is deprecated.  Use commande_locale-msg:n_poste instead.")
  (n_poste m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Msg_Erreur>) ostream)
  "Serializes a message object of type '<Msg_Erreur>"
  (cl:let* ((signed (cl:slot-value msg 'code)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'n_poste)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Msg_Erreur>) istream)
  "Deserializes a message object of type '<Msg_Erreur>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'code) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n_poste) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Msg_Erreur>)))
  "Returns string type for a message object of type '<Msg_Erreur>"
  "commande_locale/Msg_Erreur")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Msg_Erreur)))
  "Returns string type for a message object of type 'Msg_Erreur"
  "commande_locale/Msg_Erreur")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Msg_Erreur>)))
  "Returns md5sum for a message object of type '<Msg_Erreur>"
  "81853f895108ea47877f09e6d08519dc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Msg_Erreur)))
  "Returns md5sum for a message object of type 'Msg_Erreur"
  "81853f895108ea47877f09e6d08519dc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Msg_Erreur>)))
  "Returns full string definition for message of type '<Msg_Erreur>"
  (cl:format cl:nil "int32 code~%int32 n_poste~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Msg_Erreur)))
  "Returns full string definition for message of type 'Msg_Erreur"
  (cl:format cl:nil "int32 code~%int32 n_poste~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Msg_Erreur>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Msg_Erreur>))
  "Converts a ROS message object to a list"
  (cl:list 'Msg_Erreur
    (cl:cons ':code (code msg))
    (cl:cons ':n_poste (n_poste msg))
))
