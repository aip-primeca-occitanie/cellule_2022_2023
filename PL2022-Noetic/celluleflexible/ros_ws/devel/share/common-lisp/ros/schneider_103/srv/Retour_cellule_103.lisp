; Auto-generated. Do not edit!


(cl:in-package schneider_103-srv)


;//! \htmlinclude Retour_cellule_103-request.msg.html

(cl:defclass <Retour_cellule_103-request> (roslisp-msg-protocol:ros-message)
  ((memoire
    :reader memoire
    :initarg :memoire
    :type cl:integer
    :initform 0))
)

(cl:defclass Retour_cellule_103-request (<Retour_cellule_103-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Retour_cellule_103-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Retour_cellule_103-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name schneider_103-srv:<Retour_cellule_103-request> is deprecated: use schneider_103-srv:Retour_cellule_103-request instead.")))

(cl:ensure-generic-function 'memoire-val :lambda-list '(m))
(cl:defmethod memoire-val ((m <Retour_cellule_103-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:memoire-val is deprecated.  Use schneider_103-srv:memoire instead.")
  (memoire m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Retour_cellule_103-request>) ostream)
  "Serializes a message object of type '<Retour_cellule_103-request>"
  (cl:let* ((signed (cl:slot-value msg 'memoire)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Retour_cellule_103-request>) istream)
  "Deserializes a message object of type '<Retour_cellule_103-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'memoire) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Retour_cellule_103-request>)))
  "Returns string type for a service object of type '<Retour_cellule_103-request>"
  "schneider_103/Retour_cellule_103Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Retour_cellule_103-request)))
  "Returns string type for a service object of type 'Retour_cellule_103-request"
  "schneider_103/Retour_cellule_103Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Retour_cellule_103-request>)))
  "Returns md5sum for a message object of type '<Retour_cellule_103-request>"
  "144215a6005e994c4be2aa0a861077f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Retour_cellule_103-request)))
  "Returns md5sum for a message object of type 'Retour_cellule_103-request"
  "144215a6005e994c4be2aa0a861077f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Retour_cellule_103-request>)))
  "Returns full string definition for message of type '<Retour_cellule_103-request>"
  (cl:format cl:nil "int32 memoire~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Retour_cellule_103-request)))
  "Returns full string definition for message of type 'Retour_cellule_103-request"
  (cl:format cl:nil "int32 memoire~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Retour_cellule_103-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Retour_cellule_103-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Retour_cellule_103-request
    (cl:cons ':memoire (memoire msg))
))
;//! \htmlinclude Retour_cellule_103-response.msg.html

(cl:defclass <Retour_cellule_103-response> (roslisp-msg-protocol:ros-message)
  ((ST6
    :reader ST6
    :initarg :ST6
    :type cl:boolean
    :initform cl:nil)
   (ST7
    :reader ST7
    :initarg :ST7
    :type cl:boolean
    :initform cl:nil)
   (ST18
    :reader ST18
    :initarg :ST18
    :type cl:boolean
    :initform cl:nil)
   (ST19
    :reader ST19
    :initarg :ST19
    :type cl:boolean
    :initform cl:nil)
   (R3D
    :reader R3D
    :initarg :R3D
    :type cl:boolean
    :initform cl:nil)
   (R4D
    :reader R4D
    :initarg :R4D
    :type cl:boolean
    :initform cl:nil)
   (R9D
    :reader R9D
    :initarg :R9D
    :type cl:boolean
    :initform cl:nil)
   (R10D
    :reader R10D
    :initarg :R10D
    :type cl:boolean
    :initform cl:nil)
   (R3G
    :reader R3G
    :initarg :R3G
    :type cl:boolean
    :initform cl:nil)
   (R4G
    :reader R4G
    :initarg :R4G
    :type cl:boolean
    :initform cl:nil)
   (R9G
    :reader R9G
    :initarg :R9G
    :type cl:boolean
    :initform cl:nil)
   (R10G
    :reader R10G
    :initarg :R10G
    :type cl:boolean
    :initform cl:nil)
   (D3
    :reader D3
    :initarg :D3
    :type cl:boolean
    :initform cl:nil)
   (D4
    :reader D4
    :initarg :D4
    :type cl:boolean
    :initform cl:nil)
   (D9
    :reader D9
    :initarg :D9
    :type cl:boolean
    :initform cl:nil)
   (D10
    :reader D10
    :initarg :D10
    :type cl:boolean
    :initform cl:nil)
   (V3
    :reader V3
    :initarg :V3
    :type cl:boolean
    :initform cl:nil)
   (V4
    :reader V4
    :initarg :V4
    :type cl:boolean
    :initform cl:nil)
   (V9
    :reader V9
    :initarg :V9
    :type cl:boolean
    :initform cl:nil)
   (V10
    :reader V10
    :initarg :V10
    :type cl:boolean
    :initform cl:nil)
   (PS7
    :reader PS7
    :initarg :PS7
    :type cl:boolean
    :initform cl:nil)
   (PS18
    :reader PS18
    :initarg :PS18
    :type cl:boolean
    :initform cl:nil)
   (PS19
    :reader PS19
    :initarg :PS19
    :type cl:boolean
    :initform cl:nil)
   (D3D
    :reader D3D
    :initarg :D3D
    :type cl:boolean
    :initform cl:nil)
   (D4D
    :reader D4D
    :initarg :D4D
    :type cl:boolean
    :initform cl:nil)
   (D9D
    :reader D9D
    :initarg :D9D
    :type cl:boolean
    :initform cl:nil)
   (D10D
    :reader D10D
    :initarg :D10D
    :type cl:boolean
    :initform cl:nil)
   (D3G
    :reader D3G
    :initarg :D3G
    :type cl:boolean
    :initform cl:nil)
   (D4G
    :reader D4G
    :initarg :D4G
    :type cl:boolean
    :initform cl:nil)
   (D9G
    :reader D9G
    :initarg :D9G
    :type cl:boolean
    :initform cl:nil)
   (D10G
    :reader D10G
    :initarg :D10G
    :type cl:boolean
    :initform cl:nil)
   (CP3
    :reader CP3
    :initarg :CP3
    :type cl:boolean
    :initform cl:nil)
   (CP8
    :reader CP8
    :initarg :CP8
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Retour_cellule_103-response (<Retour_cellule_103-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Retour_cellule_103-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Retour_cellule_103-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name schneider_103-srv:<Retour_cellule_103-response> is deprecated: use schneider_103-srv:Retour_cellule_103-response instead.")))

(cl:ensure-generic-function 'ST6-val :lambda-list '(m))
(cl:defmethod ST6-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:ST6-val is deprecated.  Use schneider_103-srv:ST6 instead.")
  (ST6 m))

(cl:ensure-generic-function 'ST7-val :lambda-list '(m))
(cl:defmethod ST7-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:ST7-val is deprecated.  Use schneider_103-srv:ST7 instead.")
  (ST7 m))

(cl:ensure-generic-function 'ST18-val :lambda-list '(m))
(cl:defmethod ST18-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:ST18-val is deprecated.  Use schneider_103-srv:ST18 instead.")
  (ST18 m))

(cl:ensure-generic-function 'ST19-val :lambda-list '(m))
(cl:defmethod ST19-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:ST19-val is deprecated.  Use schneider_103-srv:ST19 instead.")
  (ST19 m))

(cl:ensure-generic-function 'R3D-val :lambda-list '(m))
(cl:defmethod R3D-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:R3D-val is deprecated.  Use schneider_103-srv:R3D instead.")
  (R3D m))

(cl:ensure-generic-function 'R4D-val :lambda-list '(m))
(cl:defmethod R4D-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:R4D-val is deprecated.  Use schneider_103-srv:R4D instead.")
  (R4D m))

(cl:ensure-generic-function 'R9D-val :lambda-list '(m))
(cl:defmethod R9D-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:R9D-val is deprecated.  Use schneider_103-srv:R9D instead.")
  (R9D m))

(cl:ensure-generic-function 'R10D-val :lambda-list '(m))
(cl:defmethod R10D-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:R10D-val is deprecated.  Use schneider_103-srv:R10D instead.")
  (R10D m))

(cl:ensure-generic-function 'R3G-val :lambda-list '(m))
(cl:defmethod R3G-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:R3G-val is deprecated.  Use schneider_103-srv:R3G instead.")
  (R3G m))

(cl:ensure-generic-function 'R4G-val :lambda-list '(m))
(cl:defmethod R4G-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:R4G-val is deprecated.  Use schneider_103-srv:R4G instead.")
  (R4G m))

(cl:ensure-generic-function 'R9G-val :lambda-list '(m))
(cl:defmethod R9G-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:R9G-val is deprecated.  Use schneider_103-srv:R9G instead.")
  (R9G m))

(cl:ensure-generic-function 'R10G-val :lambda-list '(m))
(cl:defmethod R10G-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:R10G-val is deprecated.  Use schneider_103-srv:R10G instead.")
  (R10G m))

(cl:ensure-generic-function 'D3-val :lambda-list '(m))
(cl:defmethod D3-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D3-val is deprecated.  Use schneider_103-srv:D3 instead.")
  (D3 m))

(cl:ensure-generic-function 'D4-val :lambda-list '(m))
(cl:defmethod D4-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D4-val is deprecated.  Use schneider_103-srv:D4 instead.")
  (D4 m))

(cl:ensure-generic-function 'D9-val :lambda-list '(m))
(cl:defmethod D9-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D9-val is deprecated.  Use schneider_103-srv:D9 instead.")
  (D9 m))

(cl:ensure-generic-function 'D10-val :lambda-list '(m))
(cl:defmethod D10-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D10-val is deprecated.  Use schneider_103-srv:D10 instead.")
  (D10 m))

(cl:ensure-generic-function 'V3-val :lambda-list '(m))
(cl:defmethod V3-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:V3-val is deprecated.  Use schneider_103-srv:V3 instead.")
  (V3 m))

(cl:ensure-generic-function 'V4-val :lambda-list '(m))
(cl:defmethod V4-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:V4-val is deprecated.  Use schneider_103-srv:V4 instead.")
  (V4 m))

(cl:ensure-generic-function 'V9-val :lambda-list '(m))
(cl:defmethod V9-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:V9-val is deprecated.  Use schneider_103-srv:V9 instead.")
  (V9 m))

(cl:ensure-generic-function 'V10-val :lambda-list '(m))
(cl:defmethod V10-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:V10-val is deprecated.  Use schneider_103-srv:V10 instead.")
  (V10 m))

(cl:ensure-generic-function 'PS7-val :lambda-list '(m))
(cl:defmethod PS7-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:PS7-val is deprecated.  Use schneider_103-srv:PS7 instead.")
  (PS7 m))

(cl:ensure-generic-function 'PS18-val :lambda-list '(m))
(cl:defmethod PS18-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:PS18-val is deprecated.  Use schneider_103-srv:PS18 instead.")
  (PS18 m))

(cl:ensure-generic-function 'PS19-val :lambda-list '(m))
(cl:defmethod PS19-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:PS19-val is deprecated.  Use schneider_103-srv:PS19 instead.")
  (PS19 m))

(cl:ensure-generic-function 'D3D-val :lambda-list '(m))
(cl:defmethod D3D-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D3D-val is deprecated.  Use schneider_103-srv:D3D instead.")
  (D3D m))

(cl:ensure-generic-function 'D4D-val :lambda-list '(m))
(cl:defmethod D4D-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D4D-val is deprecated.  Use schneider_103-srv:D4D instead.")
  (D4D m))

(cl:ensure-generic-function 'D9D-val :lambda-list '(m))
(cl:defmethod D9D-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D9D-val is deprecated.  Use schneider_103-srv:D9D instead.")
  (D9D m))

(cl:ensure-generic-function 'D10D-val :lambda-list '(m))
(cl:defmethod D10D-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D10D-val is deprecated.  Use schneider_103-srv:D10D instead.")
  (D10D m))

(cl:ensure-generic-function 'D3G-val :lambda-list '(m))
(cl:defmethod D3G-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D3G-val is deprecated.  Use schneider_103-srv:D3G instead.")
  (D3G m))

(cl:ensure-generic-function 'D4G-val :lambda-list '(m))
(cl:defmethod D4G-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D4G-val is deprecated.  Use schneider_103-srv:D4G instead.")
  (D4G m))

(cl:ensure-generic-function 'D9G-val :lambda-list '(m))
(cl:defmethod D9G-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D9G-val is deprecated.  Use schneider_103-srv:D9G instead.")
  (D9G m))

(cl:ensure-generic-function 'D10G-val :lambda-list '(m))
(cl:defmethod D10G-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:D10G-val is deprecated.  Use schneider_103-srv:D10G instead.")
  (D10G m))

(cl:ensure-generic-function 'CP3-val :lambda-list '(m))
(cl:defmethod CP3-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:CP3-val is deprecated.  Use schneider_103-srv:CP3 instead.")
  (CP3 m))

(cl:ensure-generic-function 'CP8-val :lambda-list '(m))
(cl:defmethod CP8-val ((m <Retour_cellule_103-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_103-srv:CP8-val is deprecated.  Use schneider_103-srv:CP8 instead.")
  (CP8 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Retour_cellule_103-response>) ostream)
  "Serializes a message object of type '<Retour_cellule_103-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST7) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST18) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST19) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R3D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R4D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R9D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R10D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R3G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R4G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R9G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R10G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D9) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D10) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'V3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'V4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'V9) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'V10) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS7) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS18) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS19) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D3D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D4D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D9D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D10D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D3G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D4G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D9G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D10G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CP3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CP8) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Retour_cellule_103-response>) istream)
  "Deserializes a message object of type '<Retour_cellule_103-response>"
    (cl:setf (cl:slot-value msg 'ST6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST7) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST18) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST19) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R3D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R4D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R9D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R10D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R3G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R4G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R9G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R10G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D9) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D10) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'V3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'V4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'V9) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'V10) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS7) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS18) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS19) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D3D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D4D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D9D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D10D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D3G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D4G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D9G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D10G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CP3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CP8) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Retour_cellule_103-response>)))
  "Returns string type for a service object of type '<Retour_cellule_103-response>"
  "schneider_103/Retour_cellule_103Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Retour_cellule_103-response)))
  "Returns string type for a service object of type 'Retour_cellule_103-response"
  "schneider_103/Retour_cellule_103Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Retour_cellule_103-response>)))
  "Returns md5sum for a message object of type '<Retour_cellule_103-response>"
  "144215a6005e994c4be2aa0a861077f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Retour_cellule_103-response)))
  "Returns md5sum for a message object of type 'Retour_cellule_103-response"
  "144215a6005e994c4be2aa0a861077f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Retour_cellule_103-response>)))
  "Returns full string definition for message of type '<Retour_cellule_103-response>"
  (cl:format cl:nil "~%bool ST6~%bool ST7~%bool ST18~%bool ST19~%bool R3D~%bool R4D~%bool R9D~%bool R10D~%bool R3G~%bool R4G~%bool R9G~%bool R10G~%bool D3~%bool D4~%bool D9~%bool D10~%bool V3~%bool V4~%bool V9~%bool V10~%bool PS7~%bool PS18~%bool PS19~%bool D3D~%bool D4D~%bool D9D~%bool D10D~%bool D3G~%bool D4G~%bool D9G~%bool D10G~%bool CP3~%bool CP8~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Retour_cellule_103-response)))
  "Returns full string definition for message of type 'Retour_cellule_103-response"
  (cl:format cl:nil "~%bool ST6~%bool ST7~%bool ST18~%bool ST19~%bool R3D~%bool R4D~%bool R9D~%bool R10D~%bool R3G~%bool R4G~%bool R9G~%bool R10G~%bool D3~%bool D4~%bool D9~%bool D10~%bool V3~%bool V4~%bool V9~%bool V10~%bool PS7~%bool PS18~%bool PS19~%bool D3D~%bool D4D~%bool D9D~%bool D10D~%bool D3G~%bool D4G~%bool D9G~%bool D10G~%bool CP3~%bool CP8~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Retour_cellule_103-response>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Retour_cellule_103-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Retour_cellule_103-response
    (cl:cons ':ST6 (ST6 msg))
    (cl:cons ':ST7 (ST7 msg))
    (cl:cons ':ST18 (ST18 msg))
    (cl:cons ':ST19 (ST19 msg))
    (cl:cons ':R3D (R3D msg))
    (cl:cons ':R4D (R4D msg))
    (cl:cons ':R9D (R9D msg))
    (cl:cons ':R10D (R10D msg))
    (cl:cons ':R3G (R3G msg))
    (cl:cons ':R4G (R4G msg))
    (cl:cons ':R9G (R9G msg))
    (cl:cons ':R10G (R10G msg))
    (cl:cons ':D3 (D3 msg))
    (cl:cons ':D4 (D4 msg))
    (cl:cons ':D9 (D9 msg))
    (cl:cons ':D10 (D10 msg))
    (cl:cons ':V3 (V3 msg))
    (cl:cons ':V4 (V4 msg))
    (cl:cons ':V9 (V9 msg))
    (cl:cons ':V10 (V10 msg))
    (cl:cons ':PS7 (PS7 msg))
    (cl:cons ':PS18 (PS18 msg))
    (cl:cons ':PS19 (PS19 msg))
    (cl:cons ':D3D (D3D msg))
    (cl:cons ':D4D (D4D msg))
    (cl:cons ':D9D (D9D msg))
    (cl:cons ':D10D (D10D msg))
    (cl:cons ':D3G (D3G msg))
    (cl:cons ':D4G (D4G msg))
    (cl:cons ':D9G (D9G msg))
    (cl:cons ':D10G (D10G msg))
    (cl:cons ':CP3 (CP3 msg))
    (cl:cons ':CP8 (CP8 msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Retour_cellule_103)))
  'Retour_cellule_103-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Retour_cellule_103)))
  'Retour_cellule_103-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Retour_cellule_103)))
  "Returns string type for a service object of type '<Retour_cellule_103>"
  "schneider_103/Retour_cellule_103")