;; Auto-generated. Do not edit!


(when (boundp 'commande_locale::TacheFinieMsg)
  (if (not (find-package "COMMANDE_LOCALE"))
    (make-package "COMMANDE_LOCALE"))
  (shadow 'TacheFinieMsg (find-package "COMMANDE_LOCALE")))
(unless (find-package "COMMANDE_LOCALE::TACHEFINIEMSG")
  (make-package "COMMANDE_LOCALE::TACHEFINIEMSG"))

(in-package "ROS")
;;//! \htmlinclude TacheFinieMsg.msg.html


(defclass commande_locale::TacheFinieMsg
  :super ros::object
  :slots (_num_poste _num_produit _duree ))

(defmethod commande_locale::TacheFinieMsg
  (:init
   (&key
    ((:num_poste __num_poste) 0)
    ((:num_produit __num_produit) 0)
    ((:duree __duree) 0)
    )
   (send-super :init)
   (setq _num_poste (round __num_poste))
   (setq _num_produit (round __num_produit))
   (setq _duree (round __duree))
   self)
  (:num_poste
   (&optional __num_poste)
   (if __num_poste (setq _num_poste __num_poste)) _num_poste)
  (:num_produit
   (&optional __num_produit)
   (if __num_produit (setq _num_produit __num_produit)) _num_produit)
  (:duree
   (&optional __duree)
   (if __duree (setq _duree __duree)) _duree)
  (:serialization-length
   ()
   (+
    ;; int32 _num_poste
    4
    ;; int32 _num_produit
    4
    ;; int32 _duree
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
     ;; int32 _duree
       (write-long _duree s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num_poste
     (setq _num_poste (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num_produit
     (setq _num_produit (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _duree
     (setq _duree (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get commande_locale::TacheFinieMsg :md5sum-) "4cf6cdaae99079fd26f29aa9dd559651")
(setf (get commande_locale::TacheFinieMsg :datatype-) "commande_locale/TacheFinieMsg")
(setf (get commande_locale::TacheFinieMsg :definition-)
      "int32 num_poste
int32 num_produit
int32 duree

")



(provide :commande_locale/TacheFinieMsg "4cf6cdaae99079fd26f29aa9dd559651")


