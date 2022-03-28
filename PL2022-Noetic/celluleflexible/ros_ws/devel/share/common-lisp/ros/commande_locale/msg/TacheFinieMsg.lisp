; Auto-generated. Do not edit!


(cl:in-package commande_locale-msg)


;//! \htmlinclude TacheFinieMsg.msg.html

(cl:defclass <TacheFinieMsg> (roslisp-msg-protocol:ros-message)
  ((num_poste
    :reader num_poste
    :initarg :num_poste
    :type cl:integer
    :initform 0)
   (num_produit
    :reader num_produit
    :initarg :num_produit
    :type cl:integer
    :initform 0)
   (duree
    :reader duree
    :initarg :duree
    :type cl:integer
    :initform 0))
)

(cl:defclass TacheFinieMsg (<TacheFinieMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TacheFinieMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TacheFinieMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-msg:<TacheFinieMsg> is deprecated: use commande_locale-msg:TacheFinieMsg instead.")))

(cl:ensure-generic-function 'num_poste-val :lambda-list '(m))
(cl:defmethod num_poste-val ((m <TacheFinieMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:num_poste-val is deprecated.  Use commande_locale-msg:num_poste instead.")
  (num_poste m))

(cl:ensure-generic-function 'num_produit-val :lambda-list '(m))
(cl:defmethod num_produit-val ((m <TacheFinieMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:num_produit-val is deprecated.  Use commande_locale-msg:num_produit instead.")
  (num_produit m))

(cl:ensure-generic-function 'duree-val :lambda-list '(m))
(cl:defmethod duree-val ((m <TacheFinieMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:duree-val is deprecated.  Use commande_locale-msg:duree instead.")
  (duree m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TacheFinieMsg>) ostream)
  "Serializes a message object of type '<TacheFinieMsg>"
  (cl:let* ((signed (cl:slot-value msg 'num_poste)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'num_produit)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TacheFinieMsg>) istream)
  "Deserializes a message object of type '<TacheFinieMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_poste) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_produit) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'duree) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TacheFinieMsg>)))
  "Returns string type for a message object of type '<TacheFinieMsg>"
  "commande_locale/TacheFinieMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TacheFinieMsg)))
  "Returns string type for a message object of type 'TacheFinieMsg"
  "commande_locale/TacheFinieMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TacheFinieMsg>)))
  "Returns md5sum for a message object of type '<TacheFinieMsg>"
  "4cf6cdaae99079fd26f29aa9dd559651")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TacheFinieMsg)))
  "Returns md5sum for a message object of type 'TacheFinieMsg"
  "4cf6cdaae99079fd26f29aa9dd559651")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TacheFinieMsg>)))
  "Returns full string definition for message of type '<TacheFinieMsg>"
  (cl:format cl:nil "int32 num_poste~%int32 num_produit~%int32 duree~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TacheFinieMsg)))
  "Returns full string definition for message of type 'TacheFinieMsg"
  (cl:format cl:nil "int32 num_poste~%int32 num_produit~%int32 duree~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TacheFinieMsg>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TacheFinieMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'TacheFinieMsg
    (cl:cons ':num_poste (num_poste msg))
    (cl:cons ':num_produit (num_produit msg))
    (cl:cons ':duree (duree msg))
))
