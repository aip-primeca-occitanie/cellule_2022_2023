;; Auto-generated. Do not edit!


(when (boundp 'commande_locale::SrvAddProduct)
  (if (not (find-package "COMMANDE_LOCALE"))
    (make-package "COMMANDE_LOCALE"))
  (shadow 'SrvAddProduct (find-package "COMMANDE_LOCALE")))
(unless (find-package "COMMANDE_LOCALE::SRVADDPRODUCT")
  (make-package "COMMANDE_LOCALE::SRVADDPRODUCT"))
(unless (find-package "COMMANDE_LOCALE::SRVADDPRODUCTREQUEST")
  (make-package "COMMANDE_LOCALE::SRVADDPRODUCTREQUEST"))
(unless (find-package "COMMANDE_LOCALE::SRVADDPRODUCTRESPONSE")
  (make-package "COMMANDE_LOCALE::SRVADDPRODUCTRESPONSE"))

(in-package "ROS")





(defclass commande_locale::SrvAddProductRequest
  :super ros::object
  :slots (_choixPoste _choixProduit ))

(defmethod commande_locale::SrvAddProductRequest
  (:init
   (&key
    ((:choixPoste __choixPoste) 0)
    ((:choixProduit __choixProduit) 0)
    )
   (send-super :init)
   (setq _choixPoste (round __choixPoste))
   (setq _choixProduit (round __choixProduit))
   self)
  (:choixPoste
   (&optional __choixPoste)
   (if __choixPoste (setq _choixPoste __choixPoste)) _choixPoste)
  (:choixProduit
   (&optional __choixProduit)
   (if __choixProduit (setq _choixProduit __choixProduit)) _choixProduit)
  (:serialization-length
   ()
   (+
    ;; int32 _choixPoste
    4
    ;; int32 _choixProduit
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _choixPoste
       (write-long _choixPoste s)
     ;; int32 _choixProduit
       (write-long _choixProduit s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _choixPoste
     (setq _choixPoste (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _choixProduit
     (setq _choixProduit (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass commande_locale::SrvAddProductResponse
  :super ros::object
  :slots ())

(defmethod commande_locale::SrvAddProductResponse
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

(defclass commande_locale::SrvAddProduct
  :super ros::object
  :slots ())

(setf (get commande_locale::SrvAddProduct :md5sum-) "02830da78068d9cdc27b052881f589fb")
(setf (get commande_locale::SrvAddProduct :datatype-) "commande_locale/SrvAddProduct")
(setf (get commande_locale::SrvAddProduct :request) commande_locale::SrvAddProductRequest)
(setf (get commande_locale::SrvAddProduct :response) commande_locale::SrvAddProductResponse)

(defmethod commande_locale::SrvAddProductRequest
  (:response () (instance commande_locale::SrvAddProductResponse :init)))

(setf (get commande_locale::SrvAddProductRequest :md5sum-) "02830da78068d9cdc27b052881f589fb")
(setf (get commande_locale::SrvAddProductRequest :datatype-) "commande_locale/SrvAddProductRequest")
(setf (get commande_locale::SrvAddProductRequest :definition-)
      "int32 choixPoste
int32 choixProduit
---

")

(setf (get commande_locale::SrvAddProductResponse :md5sum-) "02830da78068d9cdc27b052881f589fb")
(setf (get commande_locale::SrvAddProductResponse :datatype-) "commande_locale/SrvAddProductResponse")
(setf (get commande_locale::SrvAddProductResponse :definition-)
      "int32 choixPoste
int32 choixProduit
---

")



(provide :commande_locale/SrvAddProduct "02830da78068d9cdc27b052881f589fb")


