; Auto-generated. Do not edit!


(cl:in-package commande_locale-msg)


;//! \htmlinclude Msg_AddProduct.msg.html

(cl:defclass <Msg_AddProduct> (roslisp-msg-protocol:ros-message)
  ((num_poste
    :reader num_poste
    :initarg :num_poste
    :type cl:integer
    :initform 0)
   (num_produit
    :reader num_produit
    :initarg :num_produit
    :type cl:integer
    :initform 0))
)

(cl:defclass Msg_AddProduct (<Msg_AddProduct>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Msg_AddProduct>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Msg_AddProduct)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-msg:<Msg_AddProduct> is deprecated: use commande_locale-msg:Msg_AddProduct instead.")))

(cl:ensure-generic-function 'num_poste-val :lambda-list '(m))
(cl:defmethod num_poste-val ((m <Msg_AddProduct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:num_poste-val is deprecated.  Use commande_locale-msg:num_poste instead.")
  (num_poste m))

(cl:ensure-generic-function 'num_produit-val :lambda-list '(m))
(cl:defmethod num_produit-val ((m <Msg_AddProduct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader commande_locale-msg:num_produit-val is deprecated.  Use commande_locale-msg:num_produit instead.")
  (num_produit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Msg_AddProduct>) ostream)
  "Serializes a message object of type '<Msg_AddProduct>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Msg_AddProduct>) istream)
  "Deserializes a message object of type '<Msg_AddProduct>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Msg_AddProduct>)))
  "Returns string type for a message object of type '<Msg_AddProduct>"
  "commande_locale/Msg_AddProduct")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Msg_AddProduct)))
  "Returns string type for a message object of type 'Msg_AddProduct"
  "commande_locale/Msg_AddProduct")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Msg_AddProduct>)))
  "Returns md5sum for a message object of type '<Msg_AddProduct>"
  "d95cfaa1f3001fb14e83d571d6acf3f9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Msg_AddProduct)))
  "Returns md5sum for a message object of type 'Msg_AddProduct"
  "d95cfaa1f3001fb14e83d571d6acf3f9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Msg_AddProduct>)))
  "Returns full string definition for message of type '<Msg_AddProduct>"
  (cl:format cl:nil "int32 num_poste~%int32 num_produit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Msg_AddProduct)))
  "Returns full string definition for message of type 'Msg_AddProduct"
  (cl:format cl:nil "int32 num_poste~%int32 num_produit~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Msg_AddProduct>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Msg_AddProduct>))
  "Converts a ROS message object to a list"
  (cl:list 'Msg_AddProduct
    (cl:cons ':num_poste (num_poste msg))
    (cl:cons ':num_produit (num_produit msg))
))
