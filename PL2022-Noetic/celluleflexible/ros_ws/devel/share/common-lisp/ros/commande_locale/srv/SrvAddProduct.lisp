; Auto-generated. Do not edit!


(cl:in-package commande_locale-srv)


;//! \htmlinclude SrvAddProduct-request.msg.html

(cl:defclass <SrvAddProduct-request> (roslisp-msg-protocol:ros-message)
  ((choixPoste
    :reader choixPoste
    :initarg :choixPoste
    :type cl:integer
    :initform 0)
   (choixProduit
    :reader choixProduit
    :initarg :choixProduit
    :type cl:integer
    :initform 0))
)

(cl:defclass SrvAddProduct-request (<SrvAddProduct-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvAddProduct-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvAddProduct-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-srv:<SrvAddProduct-request> is deprecated: use commande_locale-srv:SrvAddProduct-request instead.")))

(cl:ensure-generic-function 'choixPoste-val :lambda-list '(m))
(cl:defmethod choixPoste-val ((m <SrvAddProduct-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-srv:choixPoste-val is deprecated.  Use commande_locale-srv:choixPoste instead.")
  (choixPoste m))

(cl:ensure-generic-function 'choixProduit-val :lambda-list '(m))
(cl:defmethod choixProduit-val ((m <SrvAddProduct-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-srv:choixProduit-val is deprecated.  Use commande_locale-srv:choixProduit instead.")
  (choixProduit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvAddProduct-request>) ostream)
  "Serializes a message object of type '<SrvAddProduct-request>"
  (cl:let* ((signed (cl:slot-value msg 'choixPoste)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'choixProduit)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvAddProduct-request>) istream)
  "Deserializes a message object of type '<SrvAddProduct-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'choixPoste) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'choixProduit) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvAddProduct-request>)))
  "Returns string type for a service object of type '<SrvAddProduct-request>"
  "commande_locale/SrvAddProductRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvAddProduct-request)))
  "Returns string type for a service object of type 'SrvAddProduct-request"
  "commande_locale/SrvAddProductRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvAddProduct-request>)))
  "Returns md5sum for a message object of type '<SrvAddProduct-request>"
  "02830da78068d9cdc27b052881f589fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvAddProduct-request)))
  "Returns md5sum for a message object of type 'SrvAddProduct-request"
  "02830da78068d9cdc27b052881f589fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvAddProduct-request>)))
  "Returns full string definition for message of type '<SrvAddProduct-request>"
  (cl:format cl:nil "int32 choixPoste~%int32 choixProduit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvAddProduct-request)))
  "Returns full string definition for message of type 'SrvAddProduct-request"
  (cl:format cl:nil "int32 choixPoste~%int32 choixProduit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvAddProduct-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvAddProduct-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvAddProduct-request
    (cl:cons ':choixPoste (choixPoste msg))
    (cl:cons ':choixProduit (choixProduit msg))
))
;//! \htmlinclude SrvAddProduct-response.msg.html

(cl:defclass <SrvAddProduct-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SrvAddProduct-response (<SrvAddProduct-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvAddProduct-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvAddProduct-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-srv:<SrvAddProduct-response> is deprecated: use commande_locale-srv:SrvAddProduct-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvAddProduct-response>) ostream)
  "Serializes a message object of type '<SrvAddProduct-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvAddProduct-response>) istream)
  "Deserializes a message object of type '<SrvAddProduct-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvAddProduct-response>)))
  "Returns string type for a service object of type '<SrvAddProduct-response>"
  "commande_locale/SrvAddProductResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvAddProduct-response)))
  "Returns string type for a service object of type 'SrvAddProduct-response"
  "commande_locale/SrvAddProductResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvAddProduct-response>)))
  "Returns md5sum for a message object of type '<SrvAddProduct-response>"
  "02830da78068d9cdc27b052881f589fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvAddProduct-response)))
  "Returns md5sum for a message object of type 'SrvAddProduct-response"
  "02830da78068d9cdc27b052881f589fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvAddProduct-response>)))
  "Returns full string definition for message of type '<SrvAddProduct-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvAddProduct-response)))
  "Returns full string definition for message of type 'SrvAddProduct-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvAddProduct-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvAddProduct-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvAddProduct-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SrvAddProduct)))
  'SrvAddProduct-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SrvAddProduct)))
  'SrvAddProduct-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvAddProduct)))
  "Returns string type for a service object of type '<SrvAddProduct>"
  "commande_locale/SrvAddProduct")