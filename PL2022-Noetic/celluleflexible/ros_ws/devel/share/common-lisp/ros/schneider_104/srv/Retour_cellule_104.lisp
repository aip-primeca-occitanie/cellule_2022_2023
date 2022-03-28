; Auto-generated. Do not edit!


(cl:in-package schneider_104-srv)


;//! \htmlinclude Retour_cellule_104-request.msg.html

(cl:defclass <Retour_cellule_104-request> (roslisp-msg-protocol:ros-message)
  ((memoire
    :reader memoire
    :initarg :memoire
    :type cl:integer
    :initform 0))
)

(cl:defclass Retour_cellule_104-request (<Retour_cellule_104-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Retour_cellule_104-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Retour_cellule_104-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name schneider_104-srv:<Retour_cellule_104-request> is deprecated: use schneider_104-srv:Retour_cellule_104-request instead.")))

(cl:ensure-generic-function 'memoire-val :lambda-list '(m))
(cl:defmethod memoire-val ((m <Retour_cellule_104-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:memoire-val is deprecated.  Use schneider_104-srv:memoire instead.")
  (memoire m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Retour_cellule_104-request>) ostream)
  "Serializes a message object of type '<Retour_cellule_104-request>"
  (cl:let* ((signed (cl:slot-value msg 'memoire)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Retour_cellule_104-request>) istream)
  "Deserializes a message object of type '<Retour_cellule_104-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'memoire) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Retour_cellule_104-request>)))
  "Returns string type for a service object of type '<Retour_cellule_104-request>"
  "schneider_104/Retour_cellule_104Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Retour_cellule_104-request)))
  "Returns string type for a service object of type 'Retour_cellule_104-request"
  "schneider_104/Retour_cellule_104Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Retour_cellule_104-request>)))
  "Returns md5sum for a message object of type '<Retour_cellule_104-request>"
  "da2dff768924005c2c1434c86e6e1a14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Retour_cellule_104-request)))
  "Returns md5sum for a message object of type 'Retour_cellule_104-request"
  "da2dff768924005c2c1434c86e6e1a14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Retour_cellule_104-request>)))
  "Returns full string definition for message of type '<Retour_cellule_104-request>"
  (cl:format cl:nil "int32 memoire~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Retour_cellule_104-request)))
  "Returns full string definition for message of type 'Retour_cellule_104-request"
  (cl:format cl:nil "int32 memoire~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Retour_cellule_104-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Retour_cellule_104-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Retour_cellule_104-request
    (cl:cons ':memoire (memoire msg))
))
;//! \htmlinclude Retour_cellule_104-response.msg.html

(cl:defclass <Retour_cellule_104-response> (roslisp-msg-protocol:ros-message)
  ((ST8
    :reader ST8
    :initarg :ST8
    :type cl:boolean
    :initform cl:nil)
   (ST9
    :reader ST9
    :initarg :ST9
    :type cl:boolean
    :initform cl:nil)
   (ST10
    :reader ST10
    :initarg :ST10
    :type cl:boolean
    :initform cl:nil)
   (ST11
    :reader ST11
    :initarg :ST11
    :type cl:boolean
    :initform cl:nil)
   (ST12
    :reader ST12
    :initarg :ST12
    :type cl:boolean
    :initform cl:nil)
   (ST13
    :reader ST13
    :initarg :ST13
    :type cl:boolean
    :initform cl:nil)
   (ST14
    :reader ST14
    :initarg :ST14
    :type cl:boolean
    :initform cl:nil)
   (ST15
    :reader ST15
    :initarg :ST15
    :type cl:boolean
    :initform cl:nil)
   (ST16
    :reader ST16
    :initarg :ST16
    :type cl:boolean
    :initform cl:nil)
   (ST17
    :reader ST17
    :initarg :ST17
    :type cl:boolean
    :initform cl:nil)
   (R5D
    :reader R5D
    :initarg :R5D
    :type cl:boolean
    :initform cl:nil)
   (R6D
    :reader R6D
    :initarg :R6D
    :type cl:boolean
    :initform cl:nil)
   (R7D
    :reader R7D
    :initarg :R7D
    :type cl:boolean
    :initform cl:nil)
   (R8D
    :reader R8D
    :initarg :R8D
    :type cl:boolean
    :initform cl:nil)
   (R5G
    :reader R5G
    :initarg :R5G
    :type cl:boolean
    :initform cl:nil)
   (R6G
    :reader R6G
    :initarg :R6G
    :type cl:boolean
    :initform cl:nil)
   (R7G
    :reader R7G
    :initarg :R7G
    :type cl:boolean
    :initform cl:nil)
   (R8G
    :reader R8G
    :initarg :R8G
    :type cl:boolean
    :initform cl:nil)
   (PI3
    :reader PI3
    :initarg :PI3
    :type cl:boolean
    :initform cl:nil)
   (PI4
    :reader PI4
    :initarg :PI4
    :type cl:boolean
    :initform cl:nil)
   (PI5
    :reader PI5
    :initarg :PI5
    :type cl:boolean
    :initform cl:nil)
   (PI6
    :reader PI6
    :initarg :PI6
    :type cl:boolean
    :initform cl:nil)
   (D5
    :reader D5
    :initarg :D5
    :type cl:boolean
    :initform cl:nil)
   (D6
    :reader D6
    :initarg :D6
    :type cl:boolean
    :initform cl:nil)
   (D7
    :reader D7
    :initarg :D7
    :type cl:boolean
    :initform cl:nil)
   (D8
    :reader D8
    :initarg :D8
    :type cl:boolean
    :initform cl:nil)
   (V5
    :reader V5
    :initarg :V5
    :type cl:boolean
    :initform cl:nil)
   (V6
    :reader V6
    :initarg :V6
    :type cl:boolean
    :initform cl:nil)
   (V7
    :reader V7
    :initarg :V7
    :type cl:boolean
    :initform cl:nil)
   (V8
    :reader V8
    :initarg :V8
    :type cl:boolean
    :initform cl:nil)
   (PS8
    :reader PS8
    :initarg :PS8
    :type cl:boolean
    :initform cl:nil)
   (PS9
    :reader PS9
    :initarg :PS9
    :type cl:boolean
    :initform cl:nil)
   (PS10
    :reader PS10
    :initarg :PS10
    :type cl:boolean
    :initform cl:nil)
   (PS11
    :reader PS11
    :initarg :PS11
    :type cl:boolean
    :initform cl:nil)
   (PS12
    :reader PS12
    :initarg :PS12
    :type cl:boolean
    :initform cl:nil)
   (PS13
    :reader PS13
    :initarg :PS13
    :type cl:boolean
    :initform cl:nil)
   (PS14
    :reader PS14
    :initarg :PS14
    :type cl:boolean
    :initform cl:nil)
   (PS15
    :reader PS15
    :initarg :PS15
    :type cl:boolean
    :initform cl:nil)
   (PS16
    :reader PS16
    :initarg :PS16
    :type cl:boolean
    :initform cl:nil)
   (PS17
    :reader PS17
    :initarg :PS17
    :type cl:boolean
    :initform cl:nil)
   (D5D
    :reader D5D
    :initarg :D5D
    :type cl:boolean
    :initform cl:nil)
   (D6D
    :reader D6D
    :initarg :D6D
    :type cl:boolean
    :initform cl:nil)
   (D7D
    :reader D7D
    :initarg :D7D
    :type cl:boolean
    :initform cl:nil)
   (D8D
    :reader D8D
    :initarg :D8D
    :type cl:boolean
    :initform cl:nil)
   (D5G
    :reader D5G
    :initarg :D5G
    :type cl:boolean
    :initform cl:nil)
   (D6G
    :reader D6G
    :initarg :D6G
    :type cl:boolean
    :initform cl:nil)
   (D7G
    :reader D7G
    :initarg :D7G
    :type cl:boolean
    :initform cl:nil)
   (D8G
    :reader D8G
    :initarg :D8G
    :type cl:boolean
    :initform cl:nil)
   (CPI3
    :reader CPI3
    :initarg :CPI3
    :type cl:boolean
    :initform cl:nil)
   (CPI4
    :reader CPI4
    :initarg :CPI4
    :type cl:boolean
    :initform cl:nil)
   (CPI5
    :reader CPI5
    :initarg :CPI5
    :type cl:boolean
    :initform cl:nil)
   (CPI6
    :reader CPI6
    :initarg :CPI6
    :type cl:boolean
    :initform cl:nil)
   (CP4
    :reader CP4
    :initarg :CP4
    :type cl:boolean
    :initform cl:nil)
   (CP5
    :reader CP5
    :initarg :CP5
    :type cl:boolean
    :initform cl:nil)
   (CP6
    :reader CP6
    :initarg :CP6
    :type cl:boolean
    :initform cl:nil)
   (CP7
    :reader CP7
    :initarg :CP7
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Retour_cellule_104-response (<Retour_cellule_104-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Retour_cellule_104-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Retour_cellule_104-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name schneider_104-srv:<Retour_cellule_104-response> is deprecated: use schneider_104-srv:Retour_cellule_104-response instead.")))

(cl:ensure-generic-function 'ST8-val :lambda-list '(m))
(cl:defmethod ST8-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST8-val is deprecated.  Use schneider_104-srv:ST8 instead.")
  (ST8 m))

(cl:ensure-generic-function 'ST9-val :lambda-list '(m))
(cl:defmethod ST9-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST9-val is deprecated.  Use schneider_104-srv:ST9 instead.")
  (ST9 m))

(cl:ensure-generic-function 'ST10-val :lambda-list '(m))
(cl:defmethod ST10-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST10-val is deprecated.  Use schneider_104-srv:ST10 instead.")
  (ST10 m))

(cl:ensure-generic-function 'ST11-val :lambda-list '(m))
(cl:defmethod ST11-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST11-val is deprecated.  Use schneider_104-srv:ST11 instead.")
  (ST11 m))

(cl:ensure-generic-function 'ST12-val :lambda-list '(m))
(cl:defmethod ST12-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST12-val is deprecated.  Use schneider_104-srv:ST12 instead.")
  (ST12 m))

(cl:ensure-generic-function 'ST13-val :lambda-list '(m))
(cl:defmethod ST13-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST13-val is deprecated.  Use schneider_104-srv:ST13 instead.")
  (ST13 m))

(cl:ensure-generic-function 'ST14-val :lambda-list '(m))
(cl:defmethod ST14-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST14-val is deprecated.  Use schneider_104-srv:ST14 instead.")
  (ST14 m))

(cl:ensure-generic-function 'ST15-val :lambda-list '(m))
(cl:defmethod ST15-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST15-val is deprecated.  Use schneider_104-srv:ST15 instead.")
  (ST15 m))

(cl:ensure-generic-function 'ST16-val :lambda-list '(m))
(cl:defmethod ST16-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST16-val is deprecated.  Use schneider_104-srv:ST16 instead.")
  (ST16 m))

(cl:ensure-generic-function 'ST17-val :lambda-list '(m))
(cl:defmethod ST17-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:ST17-val is deprecated.  Use schneider_104-srv:ST17 instead.")
  (ST17 m))

(cl:ensure-generic-function 'R5D-val :lambda-list '(m))
(cl:defmethod R5D-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:R5D-val is deprecated.  Use schneider_104-srv:R5D instead.")
  (R5D m))

(cl:ensure-generic-function 'R6D-val :lambda-list '(m))
(cl:defmethod R6D-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:R6D-val is deprecated.  Use schneider_104-srv:R6D instead.")
  (R6D m))

(cl:ensure-generic-function 'R7D-val :lambda-list '(m))
(cl:defmethod R7D-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:R7D-val is deprecated.  Use schneider_104-srv:R7D instead.")
  (R7D m))

(cl:ensure-generic-function 'R8D-val :lambda-list '(m))
(cl:defmethod R8D-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:R8D-val is deprecated.  Use schneider_104-srv:R8D instead.")
  (R8D m))

(cl:ensure-generic-function 'R5G-val :lambda-list '(m))
(cl:defmethod R5G-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:R5G-val is deprecated.  Use schneider_104-srv:R5G instead.")
  (R5G m))

(cl:ensure-generic-function 'R6G-val :lambda-list '(m))
(cl:defmethod R6G-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:R6G-val is deprecated.  Use schneider_104-srv:R6G instead.")
  (R6G m))

(cl:ensure-generic-function 'R7G-val :lambda-list '(m))
(cl:defmethod R7G-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:R7G-val is deprecated.  Use schneider_104-srv:R7G instead.")
  (R7G m))

(cl:ensure-generic-function 'R8G-val :lambda-list '(m))
(cl:defmethod R8G-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:R8G-val is deprecated.  Use schneider_104-srv:R8G instead.")
  (R8G m))

(cl:ensure-generic-function 'PI3-val :lambda-list '(m))
(cl:defmethod PI3-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PI3-val is deprecated.  Use schneider_104-srv:PI3 instead.")
  (PI3 m))

(cl:ensure-generic-function 'PI4-val :lambda-list '(m))
(cl:defmethod PI4-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PI4-val is deprecated.  Use schneider_104-srv:PI4 instead.")
  (PI4 m))

(cl:ensure-generic-function 'PI5-val :lambda-list '(m))
(cl:defmethod PI5-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PI5-val is deprecated.  Use schneider_104-srv:PI5 instead.")
  (PI5 m))

(cl:ensure-generic-function 'PI6-val :lambda-list '(m))
(cl:defmethod PI6-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PI6-val is deprecated.  Use schneider_104-srv:PI6 instead.")
  (PI6 m))

(cl:ensure-generic-function 'D5-val :lambda-list '(m))
(cl:defmethod D5-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D5-val is deprecated.  Use schneider_104-srv:D5 instead.")
  (D5 m))

(cl:ensure-generic-function 'D6-val :lambda-list '(m))
(cl:defmethod D6-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D6-val is deprecated.  Use schneider_104-srv:D6 instead.")
  (D6 m))

(cl:ensure-generic-function 'D7-val :lambda-list '(m))
(cl:defmethod D7-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D7-val is deprecated.  Use schneider_104-srv:D7 instead.")
  (D7 m))

(cl:ensure-generic-function 'D8-val :lambda-list '(m))
(cl:defmethod D8-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D8-val is deprecated.  Use schneider_104-srv:D8 instead.")
  (D8 m))

(cl:ensure-generic-function 'V5-val :lambda-list '(m))
(cl:defmethod V5-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:V5-val is deprecated.  Use schneider_104-srv:V5 instead.")
  (V5 m))

(cl:ensure-generic-function 'V6-val :lambda-list '(m))
(cl:defmethod V6-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:V6-val is deprecated.  Use schneider_104-srv:V6 instead.")
  (V6 m))

(cl:ensure-generic-function 'V7-val :lambda-list '(m))
(cl:defmethod V7-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:V7-val is deprecated.  Use schneider_104-srv:V7 instead.")
  (V7 m))

(cl:ensure-generic-function 'V8-val :lambda-list '(m))
(cl:defmethod V8-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:V8-val is deprecated.  Use schneider_104-srv:V8 instead.")
  (V8 m))

(cl:ensure-generic-function 'PS8-val :lambda-list '(m))
(cl:defmethod PS8-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS8-val is deprecated.  Use schneider_104-srv:PS8 instead.")
  (PS8 m))

(cl:ensure-generic-function 'PS9-val :lambda-list '(m))
(cl:defmethod PS9-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS9-val is deprecated.  Use schneider_104-srv:PS9 instead.")
  (PS9 m))

(cl:ensure-generic-function 'PS10-val :lambda-list '(m))
(cl:defmethod PS10-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS10-val is deprecated.  Use schneider_104-srv:PS10 instead.")
  (PS10 m))

(cl:ensure-generic-function 'PS11-val :lambda-list '(m))
(cl:defmethod PS11-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS11-val is deprecated.  Use schneider_104-srv:PS11 instead.")
  (PS11 m))

(cl:ensure-generic-function 'PS12-val :lambda-list '(m))
(cl:defmethod PS12-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS12-val is deprecated.  Use schneider_104-srv:PS12 instead.")
  (PS12 m))

(cl:ensure-generic-function 'PS13-val :lambda-list '(m))
(cl:defmethod PS13-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS13-val is deprecated.  Use schneider_104-srv:PS13 instead.")
  (PS13 m))

(cl:ensure-generic-function 'PS14-val :lambda-list '(m))
(cl:defmethod PS14-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS14-val is deprecated.  Use schneider_104-srv:PS14 instead.")
  (PS14 m))

(cl:ensure-generic-function 'PS15-val :lambda-list '(m))
(cl:defmethod PS15-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS15-val is deprecated.  Use schneider_104-srv:PS15 instead.")
  (PS15 m))

(cl:ensure-generic-function 'PS16-val :lambda-list '(m))
(cl:defmethod PS16-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS16-val is deprecated.  Use schneider_104-srv:PS16 instead.")
  (PS16 m))

(cl:ensure-generic-function 'PS17-val :lambda-list '(m))
(cl:defmethod PS17-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:PS17-val is deprecated.  Use schneider_104-srv:PS17 instead.")
  (PS17 m))

(cl:ensure-generic-function 'D5D-val :lambda-list '(m))
(cl:defmethod D5D-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D5D-val is deprecated.  Use schneider_104-srv:D5D instead.")
  (D5D m))

(cl:ensure-generic-function 'D6D-val :lambda-list '(m))
(cl:defmethod D6D-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D6D-val is deprecated.  Use schneider_104-srv:D6D instead.")
  (D6D m))

(cl:ensure-generic-function 'D7D-val :lambda-list '(m))
(cl:defmethod D7D-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D7D-val is deprecated.  Use schneider_104-srv:D7D instead.")
  (D7D m))

(cl:ensure-generic-function 'D8D-val :lambda-list '(m))
(cl:defmethod D8D-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D8D-val is deprecated.  Use schneider_104-srv:D8D instead.")
  (D8D m))

(cl:ensure-generic-function 'D5G-val :lambda-list '(m))
(cl:defmethod D5G-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D5G-val is deprecated.  Use schneider_104-srv:D5G instead.")
  (D5G m))

(cl:ensure-generic-function 'D6G-val :lambda-list '(m))
(cl:defmethod D6G-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D6G-val is deprecated.  Use schneider_104-srv:D6G instead.")
  (D6G m))

(cl:ensure-generic-function 'D7G-val :lambda-list '(m))
(cl:defmethod D7G-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D7G-val is deprecated.  Use schneider_104-srv:D7G instead.")
  (D7G m))

(cl:ensure-generic-function 'D8G-val :lambda-list '(m))
(cl:defmethod D8G-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:D8G-val is deprecated.  Use schneider_104-srv:D8G instead.")
  (D8G m))

(cl:ensure-generic-function 'CPI3-val :lambda-list '(m))
(cl:defmethod CPI3-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:CPI3-val is deprecated.  Use schneider_104-srv:CPI3 instead.")
  (CPI3 m))

(cl:ensure-generic-function 'CPI4-val :lambda-list '(m))
(cl:defmethod CPI4-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:CPI4-val is deprecated.  Use schneider_104-srv:CPI4 instead.")
  (CPI4 m))

(cl:ensure-generic-function 'CPI5-val :lambda-list '(m))
(cl:defmethod CPI5-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:CPI5-val is deprecated.  Use schneider_104-srv:CPI5 instead.")
  (CPI5 m))

(cl:ensure-generic-function 'CPI6-val :lambda-list '(m))
(cl:defmethod CPI6-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:CPI6-val is deprecated.  Use schneider_104-srv:CPI6 instead.")
  (CPI6 m))

(cl:ensure-generic-function 'CP4-val :lambda-list '(m))
(cl:defmethod CP4-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:CP4-val is deprecated.  Use schneider_104-srv:CP4 instead.")
  (CP4 m))

(cl:ensure-generic-function 'CP5-val :lambda-list '(m))
(cl:defmethod CP5-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:CP5-val is deprecated.  Use schneider_104-srv:CP5 instead.")
  (CP5 m))

(cl:ensure-generic-function 'CP6-val :lambda-list '(m))
(cl:defmethod CP6-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:CP6-val is deprecated.  Use schneider_104-srv:CP6 instead.")
  (CP6 m))

(cl:ensure-generic-function 'CP7-val :lambda-list '(m))
(cl:defmethod CP7-val ((m <Retour_cellule_104-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader schneider_104-srv:CP7-val is deprecated.  Use schneider_104-srv:CP7 instead.")
  (CP7 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Retour_cellule_104-response>) ostream)
  "Serializes a message object of type '<Retour_cellule_104-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST8) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST9) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST10) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST11) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST12) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST13) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST14) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST15) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST16) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ST17) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R5D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R6D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R7D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R8D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R5G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R6G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R7G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'R8G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PI3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PI4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PI5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PI6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D7) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D8) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'V5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'V6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'V7) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'V8) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS8) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS9) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS10) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS11) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS12) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS13) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS14) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS15) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS16) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'PS17) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D5D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D6D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D7D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D8D) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D5G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D6G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D7G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'D8G) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CPI3) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CPI4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CPI5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CPI6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CP4) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CP5) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CP6) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'CP7) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Retour_cellule_104-response>) istream)
  "Deserializes a message object of type '<Retour_cellule_104-response>"
    (cl:setf (cl:slot-value msg 'ST8) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST9) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST10) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST11) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST12) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST13) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST14) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST15) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST16) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ST17) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R5D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R6D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R7D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R8D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R5G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R6G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R7G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'R8G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PI3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PI4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PI5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PI6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D7) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D8) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'V5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'V6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'V7) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'V8) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS8) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS9) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS10) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS11) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS12) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS13) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS14) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS15) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS16) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'PS17) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D5D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D6D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D7D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D8D) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D5G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D6G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D7G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'D8G) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CPI3) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CPI4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CPI5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CPI6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CP4) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CP5) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CP6) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'CP7) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Retour_cellule_104-response>)))
  "Returns string type for a service object of type '<Retour_cellule_104-response>"
  "schneider_104/Retour_cellule_104Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Retour_cellule_104-response)))
  "Returns string type for a service object of type 'Retour_cellule_104-response"
  "schneider_104/Retour_cellule_104Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Retour_cellule_104-response>)))
  "Returns md5sum for a message object of type '<Retour_cellule_104-response>"
  "da2dff768924005c2c1434c86e6e1a14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Retour_cellule_104-response)))
  "Returns md5sum for a message object of type 'Retour_cellule_104-response"
  "da2dff768924005c2c1434c86e6e1a14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Retour_cellule_104-response>)))
  "Returns full string definition for message of type '<Retour_cellule_104-response>"
  (cl:format cl:nil "bool ST8~%bool ST9~%bool ST10~%bool ST11~%bool ST12~%bool ST13~%bool ST14~%bool ST15~%bool ST16~%bool ST17~%bool R5D~%bool R6D~%bool R7D~%bool R8D~%bool R5G~%bool R6G~%bool R7G~%bool R8G~%bool PI3~%bool PI4~%bool PI5~%bool PI6~%bool D5~%bool D6~%bool D7~%bool D8~%bool V5~%bool V6~%bool V7~%bool V8~%bool PS8~%bool PS9~%bool PS10~%bool PS11~%bool PS12~%bool PS13~%bool PS14~%bool PS15~%bool PS16~%bool PS17~%bool D5D~%bool D6D~%bool D7D~%bool D8D~%bool D5G~%bool D6G~%bool D7G~%bool D8G~%bool CPI3~%bool CPI4~%bool CPI5~%bool CPI6~%bool CP4~%bool CP5~%bool CP6~%bool CP7~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Retour_cellule_104-response)))
  "Returns full string definition for message of type 'Retour_cellule_104-response"
  (cl:format cl:nil "bool ST8~%bool ST9~%bool ST10~%bool ST11~%bool ST12~%bool ST13~%bool ST14~%bool ST15~%bool ST16~%bool ST17~%bool R5D~%bool R6D~%bool R7D~%bool R8D~%bool R5G~%bool R6G~%bool R7G~%bool R8G~%bool PI3~%bool PI4~%bool PI5~%bool PI6~%bool D5~%bool D6~%bool D7~%bool D8~%bool V5~%bool V6~%bool V7~%bool V8~%bool PS8~%bool PS9~%bool PS10~%bool PS11~%bool PS12~%bool PS13~%bool PS14~%bool PS15~%bool PS16~%bool PS17~%bool D5D~%bool D6D~%bool D7D~%bool D8D~%bool D5G~%bool D6G~%bool D7G~%bool D8G~%bool CPI3~%bool CPI4~%bool CPI5~%bool CPI6~%bool CP4~%bool CP5~%bool CP6~%bool CP7~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Retour_cellule_104-response>))
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
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Retour_cellule_104-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Retour_cellule_104-response
    (cl:cons ':ST8 (ST8 msg))
    (cl:cons ':ST9 (ST9 msg))
    (cl:cons ':ST10 (ST10 msg))
    (cl:cons ':ST11 (ST11 msg))
    (cl:cons ':ST12 (ST12 msg))
    (cl:cons ':ST13 (ST13 msg))
    (cl:cons ':ST14 (ST14 msg))
    (cl:cons ':ST15 (ST15 msg))
    (cl:cons ':ST16 (ST16 msg))
    (cl:cons ':ST17 (ST17 msg))
    (cl:cons ':R5D (R5D msg))
    (cl:cons ':R6D (R6D msg))
    (cl:cons ':R7D (R7D msg))
    (cl:cons ':R8D (R8D msg))
    (cl:cons ':R5G (R5G msg))
    (cl:cons ':R6G (R6G msg))
    (cl:cons ':R7G (R7G msg))
    (cl:cons ':R8G (R8G msg))
    (cl:cons ':PI3 (PI3 msg))
    (cl:cons ':PI4 (PI4 msg))
    (cl:cons ':PI5 (PI5 msg))
    (cl:cons ':PI6 (PI6 msg))
    (cl:cons ':D5 (D5 msg))
    (cl:cons ':D6 (D6 msg))
    (cl:cons ':D7 (D7 msg))
    (cl:cons ':D8 (D8 msg))
    (cl:cons ':V5 (V5 msg))
    (cl:cons ':V6 (V6 msg))
    (cl:cons ':V7 (V7 msg))
    (cl:cons ':V8 (V8 msg))
    (cl:cons ':PS8 (PS8 msg))
    (cl:cons ':PS9 (PS9 msg))
    (cl:cons ':PS10 (PS10 msg))
    (cl:cons ':PS11 (PS11 msg))
    (cl:cons ':PS12 (PS12 msg))
    (cl:cons ':PS13 (PS13 msg))
    (cl:cons ':PS14 (PS14 msg))
    (cl:cons ':PS15 (PS15 msg))
    (cl:cons ':PS16 (PS16 msg))
    (cl:cons ':PS17 (PS17 msg))
    (cl:cons ':D5D (D5D msg))
    (cl:cons ':D6D (D6D msg))
    (cl:cons ':D7D (D7D msg))
    (cl:cons ':D8D (D8D msg))
    (cl:cons ':D5G (D5G msg))
    (cl:cons ':D6G (D6G msg))
    (cl:cons ':D7G (D7G msg))
    (cl:cons ':D8G (D8G msg))
    (cl:cons ':CPI3 (CPI3 msg))
    (cl:cons ':CPI4 (CPI4 msg))
    (cl:cons ':CPI5 (CPI5 msg))
    (cl:cons ':CPI6 (CPI6 msg))
    (cl:cons ':CP4 (CP4 msg))
    (cl:cons ':CP5 (CP5 msg))
    (cl:cons ':CP6 (CP6 msg))
    (cl:cons ':CP7 (CP7 msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Retour_cellule_104)))
  'Retour_cellule_104-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Retour_cellule_104)))
  'Retour_cellule_104-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Retour_cellule_104)))
  "Returns string type for a service object of type '<Retour_cellule_104>"
  "schneider_104/Retour_cellule_104")