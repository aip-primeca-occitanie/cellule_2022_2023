; Auto-generated. Do not edit!


(cl:in-package commande_locale-srv)


;//! \htmlinclude SrvAddProductPushBack-request.msg.html

(cl:defclass <SrvAddProductPushBack-request> (roslisp-msg-protocol:ros-message)
  ((poste
    :reader poste
    :initarg :poste
    :type cl:integer
    :initform 0)
   (produit
    :reader produit
    :initarg :produit
    :type cl:integer
    :initform 0))
)

(cl:defclass SrvAddProductPushBack-request (<SrvAddProductPushBack-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvAddProductPushBack-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvAddProductPushBack-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-srv:<SrvAddProductPushBack-request> is deprecated: use commande_locale-srv:SrvAddProductPushBack-request instead.")))

(cl:ensure-generic-function 'poste-val :lambda-list '(m))
(cl:defmethod poste-val ((m <SrvAddProductPushBack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-srv:poste-val is deprecated.  Use commande_locale-srv:poste instead.")
  (poste m))

(cl:ensure-generic-function 'produit-val :lambda-list '(m))
(cl:defmethod produit-val ((m <SrvAddProductPushBack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-srv:produit-val is deprecated.  Use commande_locale-srv:produit instead.")
  (produit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvAddProductPushBack-request>) ostream)
  "Serializes a message object of type '<SrvAddProductPushBack-request>"
  (cl:let* ((signed (cl:slot-value msg 'poste)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'produit)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvAddProductPushBack-request>) istream)
  "Deserializes a message object of type '<SrvAddProductPushBack-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'poste) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'produit) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvAddProductPushBack-request>)))
  "Returns string type for a service object of type '<SrvAddProductPushBack-request>"
  "commande_locale/SrvAddProductPushBackRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvAddProductPushBack-request)))
  "Returns string type for a service object of type 'SrvAddProductPushBack-request"
  "commande_locale/SrvAddProductPushBackRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvAddProductPushBack-request>)))
  "Returns md5sum for a message object of type '<SrvAddProductPushBack-request>"
  "3039ca7d9d039656971a53824ad295d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvAddProductPushBack-request)))
  "Returns md5sum for a message object of type 'SrvAddProductPushBack-request"
  "3039ca7d9d039656971a53824ad295d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvAddProductPushBack-request>)))
  "Returns full string definition for message of type '<SrvAddProductPushBack-request>"
  (cl:format cl:nil "int32 poste~%int32 produit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvAddProductPushBack-request)))
  "Returns full string definition for message of type 'SrvAddProductPushBack-request"
  (cl:format cl:nil "int32 poste~%int32 produit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvAddProductPushBack-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvAddProductPushBack-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvAddProductPushBack-request
    (cl:cons ':poste (poste msg))
    (cl:cons ':produit (produit msg))
))
;//! \htmlinclude SrvAddProductPushBack-response.msg.html

(cl:defclass <SrvAddProductPushBack-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SrvAddProductPushBack-response (<SrvAddProductPushBack-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvAddProductPushBack-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvAddProductPushBack-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-srv:<SrvAddProductPushBack-response> is deprecated: use commande_locale-srv:SrvAddProductPushBack-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvAddProductPushBack-response>) ostream)
  "Serializes a message object of type '<SrvAddProductPushBack-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvAddProductPushBack-response>) istream)
  "Deserializes a message object of type '<SrvAddProductPushBack-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvAddProductPushBack-response>)))
  "Returns string type for a service object of type '<SrvAddProductPushBack-response>"
  "commande_locale/SrvAddProductPushBackResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvAddProductPushBack-response)))
  "Returns string type for a service object of type 'SrvAddProductPushBack-response"
  "commande_locale/SrvAddProductPushBackResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvAddProductPushBack-response>)))
  "Returns md5sum for a message object of type '<SrvAddProductPushBack-response>"
  "3039ca7d9d039656971a53824ad295d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvAddProductPushBack-response)))
  "Returns md5sum for a message object of type 'SrvAddProductPushBack-response"
  "3039ca7d9d039656971a53824ad295d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvAddProductPushBack-response>)))
  "Returns full string definition for message of type '<SrvAddProductPushBack-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvAddProductPushBack-response)))
  "Returns full string definition for message of type 'SrvAddProductPushBack-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvAddProductPushBack-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvAddProductPushBack-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvAddProductPushBack-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SrvAddProductPushBack)))
  'SrvAddProductPushBack-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SrvAddProductPushBack)))
  'SrvAddProductPushBack-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvAddProductPushBack)))
  "Returns string type for a service object of type '<SrvAddProductPushBack>"
  "commande_locale/SrvAddProductPushBack")