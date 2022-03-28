; Auto-generated. Do not edit!


(cl:in-package commande_locale-srv)


;//! \htmlinclude SrvFinInit-request.msg.html

(cl:defclass <SrvFinInit-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SrvFinInit-request (<SrvFinInit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvFinInit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvFinInit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-srv:<SrvFinInit-request> is deprecated: use commande_locale-srv:SrvFinInit-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvFinInit-request>) ostream)
  "Serializes a message object of type '<SrvFinInit-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvFinInit-request>) istream)
  "Deserializes a message object of type '<SrvFinInit-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvFinInit-request>)))
  "Returns string type for a service object of type '<SrvFinInit-request>"
  "commande_locale/SrvFinInitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvFinInit-request)))
  "Returns string type for a service object of type 'SrvFinInit-request"
  "commande_locale/SrvFinInitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvFinInit-request>)))
  "Returns md5sum for a message object of type '<SrvFinInit-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvFinInit-request)))
  "Returns md5sum for a message object of type 'SrvFinInit-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvFinInit-request>)))
  "Returns full string definition for message of type '<SrvFinInit-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvFinInit-request)))
  "Returns full string definition for message of type 'SrvFinInit-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvFinInit-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvFinInit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvFinInit-request
))
;//! \htmlinclude SrvFinInit-response.msg.html

(cl:defclass <SrvFinInit-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SrvFinInit-response (<SrvFinInit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SrvFinInit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SrvFinInit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name commande_locale-srv:<SrvFinInit-response> is deprecated: use commande_locale-srv:SrvFinInit-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SrvFinInit-response>) ostream)
  "Serializes a message object of type '<SrvFinInit-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SrvFinInit-response>) istream)
  "Deserializes a message object of type '<SrvFinInit-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SrvFinInit-response>)))
  "Returns string type for a service object of type '<SrvFinInit-response>"
  "commande_locale/SrvFinInitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvFinInit-response)))
  "Returns string type for a service object of type 'SrvFinInit-response"
  "commande_locale/SrvFinInitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SrvFinInit-response>)))
  "Returns md5sum for a message object of type '<SrvFinInit-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SrvFinInit-response)))
  "Returns md5sum for a message object of type 'SrvFinInit-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SrvFinInit-response>)))
  "Returns full string definition for message of type '<SrvFinInit-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SrvFinInit-response)))
  "Returns full string definition for message of type 'SrvFinInit-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SrvFinInit-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SrvFinInit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SrvFinInit-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SrvFinInit)))
  'SrvFinInit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SrvFinInit)))
  'SrvFinInit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SrvFinInit)))
  "Returns string type for a service object of type '<SrvFinInit>"
  "commande_locale/SrvFinInit")