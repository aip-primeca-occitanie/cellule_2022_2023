;; Auto-generated. Do not edit!


(when (boundp 'commande_locale::SrvFinInit)
  (if (not (find-package "COMMANDE_LOCALE"))
    (make-package "COMMANDE_LOCALE"))
  (shadow 'SrvFinInit (find-package "COMMANDE_LOCALE")))
(unless (find-package "COMMANDE_LOCALE::SRVFININIT")
  (make-package "COMMANDE_LOCALE::SRVFININIT"))
(unless (find-package "COMMANDE_LOCALE::SRVFININITREQUEST")
  (make-package "COMMANDE_LOCALE::SRVFININITREQUEST"))
(unless (find-package "COMMANDE_LOCALE::SRVFININITRESPONSE")
  (make-package "COMMANDE_LOCALE::SRVFININITRESPONSE"))

(in-package "ROS")





(defclass commande_locale::SrvFinInitRequest
  :super ros::object
  :slots ())

(defmethod commande_locale::SrvFinInitRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass commande_locale::SrvFinInitResponse
  :super ros::object
  :slots ())

(defmethod commande_locale::SrvFinInitResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass commande_locale::SrvFinInit
  :super ros::object
  :slots ())

(setf (get commande_locale::SrvFinInit :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get commande_locale::SrvFinInit :datatype-) "commande_locale/SrvFinInit")
(setf (get commande_locale::SrvFinInit :request) commande_locale::SrvFinInitRequest)
(setf (get commande_locale::SrvFinInit :response) commande_locale::SrvFinInitResponse)

(defmethod commande_locale::SrvFinInitRequest
  (:response () (instance commande_locale::SrvFinInitResponse :init)))

(setf (get commande_locale::SrvFinInitRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get commande_locale::SrvFinInitRequest :datatype-) "commande_locale/SrvFinInitRequest")
(setf (get commande_locale::SrvFinInitRequest :definition-)
      "---

")

(setf (get commande_locale::SrvFinInitResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get commande_locale::SrvFinInitResponse :datatype-) "commande_locale/SrvFinInitResponse")
(setf (get commande_locale::SrvFinInitResponse :definition-)
      "---

")



(provide :commande_locale/SrvFinInit "d41d8cd98f00b204e9800998ecf8427e")


