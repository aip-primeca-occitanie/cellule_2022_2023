;; Auto-generated. Do not edit!


(when (boundp 'commande_locale::DeplacerPieceMsg)
  (if (not (find-package "COMMANDE_LOCALE"))
    (make-package "COMMANDE_LOCALE"))
  (shadow 'DeplacerPieceMsg (find-package "COMMANDE_LOCALE")))
(unless (find-package "COMMANDE_LOCALE::DEPLACERPIECEMSG")
  (make-package "COMMANDE_LOCALE::DEPLACERPIECEMSG"))

(in-package "ROS")
;;//! \htmlinclude DeplacerPieceMsg.msg.html


(defclass commande_locale::DeplacerPieceMsg
  :super ros::object
  :slots (_num_robot _positionA _positionB ))

(defmethod commande_locale::DeplacerPieceMsg
  (:init
   (&key
    ((:num_robot __num_robot) 0)
    ((:positionA __positionA) 0)
    ((:positionB __positionB) 0)
    )
   (send-super :init)
   (setq _num_robot (round __num_robot))
   (setq _positionA (round __positionA))
   (setq _positionB (round __positionB))
   self)
  (:num_robot
   (&optional __num_robot)
   (if __num_robot (setq _num_robot __num_robot)) _num_robot)
  (:positionA
   (&optional __positionA)
   (if __positionA (setq _positionA __positionA)) _positionA)
  (:positionB
   (&optional __positionB)
   (if __positionB (setq _positionB __positionB)) _positionB)
  (:serialization-length
   ()
   (+
    ;; int32 _num_robot
    4
    ;; int32 _positionA
    4
    ;; int32 _positionB
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num_robot
       (write-long _num_robot s)
     ;; int32 _positionA
       (write-long _positionA s)
     ;; int32 _positionB
       (write-long _positionB s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num_robot
     (setq _num_robot (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _positionA
     (setq _positionA (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _positionB
     (setq _positionB (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get commande_locale::DeplacerPieceMsg :md5sum-) "3590f7e74b621fec7039b4d9aecf39ba")
(setf (get commande_locale::DeplacerPieceMsg :datatype-) "commande_locale/DeplacerPieceMsg")
(setf (get commande_locale::DeplacerPieceMsg :definition-)
      "int32 num_robot
int32 positionA
int32 positionB

")



(provide :commande_locale/DeplacerPieceMsg "3590f7e74b621fec7039b4d9aecf39ba")


