;; Auto-generated. Do not edit!


(when (boundp 'commande_locale::Msg_AddProduct)
  (if (not (find-package "COMMANDE_LOCALE"))
    (make-package "COMMANDE_LOCALE"))
  (shadow 'Msg_AddProduct (find-package "COMMANDE_LOCALE")))
(unless (find-package "COMMANDE_LOCALE::MSG_ADDPRODUCT")
  (make-package "COMMANDE_LOCALE::MSG_ADDPRODUCT"))

(in-package "ROS")
;;//! \htmlinclude Msg_AddProduct.msg.html


(defclass commande_locale::Msg_AddProduct
  :super ros::object
  :slots (_num_poste _num_produit ))

(defmethod commande_locale::Msg_AddProduct
  (:init
   (&key
    ((:num_poste __num_poste) 0)
    ((:num_produit __num_produit) 0)
    )
   (send-super :init)
   (setq _num_poste (round __num_poste))
   (setq _num_produit (round __num_produit))
   self)
  (:num_poste
   (&optional __num_poste)
   (if __num_poste (setq _num_poste __num_poste)) _num_poste)
  (:num_produit
   (&optional __num_produit)
   (if __num_produit (setq _num_produit __num_produit)) _num_produit)
  (:serialization-length
   ()
   (+
    ;; int32 _num_poste
    4
    ;; int32 _num_produit
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num_poste
       (write-long _num_poste s)
     ;; int32 _num_produit
       (write-long _num_produit s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num_poste
     (setq _num_poste (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_produit
     (setq _num_produit (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get commande_locale::Msg_AddProduct :md5sum-) "d95cfaa1f3001fb14e83d571d6acf3f9")
(setf (get commande_locale::Msg_AddProduct :datatype-) "commande_locale/Msg_AddProduct")
(setf (get commande_locale::Msg_AddProduct :definition-)
      "int32 num_poste
int32 num_produit

")



(provide :commande_locale/Msg_AddProduct "d95cfaa1f3001fb14e83d571d6acf3f9")


