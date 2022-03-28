;; Auto-generated. Do not edit!


(when (boundp 'commande_locale::SrvAddProductPushBack)
  (if (not (find-package "COMMANDE_LOCALE"))
    (make-package "COMMANDE_LOCALE"))
  (shadow 'SrvAddProductPushBack (find-package "COMMANDE_LOCALE")))
(unless (find-package "COMMANDE_LOCALE::SRVADDPRODUCTPUSHBACK")
  (make-package "COMMANDE_LOCALE::SRVADDPRODUCTPUSHBACK"))
(unless (find-package "COMMANDE_LOCALE::SRVADDPRODUCTPUSHBACKREQUEST")
  (make-package "COMMANDE_LOCALE::SRVADDPRODUCTPUSHBACKREQUEST"))
(unless (find-package "COMMANDE_LOCALE::SRVADDPRODUCTPUSHBACKRESPONSE")
  (make-package "COMMANDE_LOCALE::SRVADDPRODUCTPUSHBACKRESPONSE"))

(in-package "ROS")





(defclass commande_locale::SrvAddProductPushBackRequest
  :super ros::object
  :slots (_poste _produit ))

(defmethod commande_locale::SrvAddProductPushBackRequest
  (:init
   (&key
    ((:poste __poste) 0)
    ((:produit __produit) 0)
    )
   (send-super :init)
   (setq _poste (round __poste))
   (setq _produit (round __produit))
   self)
  (:poste
   (&optional __poste)
   (if __poste (setq _poste __poste)) _poste)
  (:produit
   (&optional __produit)
   (if __produit (setq _produit __produit)) _produit)
  (:serialization-length
   ()
   (+
    ;; int32 _poste
    4
    ;; int32 _produit
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _poste
       (write-long _poste s)
     ;; int32 _produit
       (write-long _produit s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _poste
     (setq _poste (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _produit
     (setq _produit (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass commande_locale::SrvAddProductPushBackResponse
  :super ros::object
  :slots ())

(defmethod commande_locale::SrvAddProductPushBackResponse
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

(defclass commande_locale::SrvAddProductPushBack
  :super ros::object
  :slots ())

(setf (get commande_locale::SrvAddProductPushBack :md5sum-) "3039ca7d9d039656971a53824ad295d1")
(setf (get commande_locale::SrvAddProductPushBack :datatype-) "commande_locale/SrvAddProductPushBack")
(setf (get commande_locale::SrvAddProductPushBack :request) commande_locale::SrvAddProductPushBackRequest)
(setf (get commande_locale::SrvAddProductPushBack :response) commande_locale::SrvAddProductPushBackResponse)

(defmethod commande_locale::SrvAddProductPushBackRequest
  (:response () (instance commande_locale::SrvAddProductPushBackResponse :init)))

(setf (get commande_locale::SrvAddProductPushBackRequest :md5sum-) "3039ca7d9d039656971a53824ad295d1")
(setf (get commande_locale::SrvAddProductPushBackRequest :datatype-) "commande_locale/SrvAddProductPushBackRequest")
(setf (get commande_locale::SrvAddProductPushBackRequest :definition-)
      "int32 poste
int32 produit
---

")

(setf (get commande_locale::SrvAddProductPushBackResponse :md5sum-) "3039ca7d9d039656971a53824ad295d1")
(setf (get commande_locale::SrvAddProductPushBackResponse :datatype-) "commande_locale/SrvAddProductPushBackResponse")
(setf (get commande_locale::SrvAddProductPushBackResponse :definition-)
      "int32 poste
int32 produit
---

")



(provide :commande_locale/SrvAddProductPushBack "3039ca7d9d039656971a53824ad295d1")


