;; Auto-generated. Do not edit!


(when (boundp 'schneider_103::Retour_cellule_103)
  (if (not (find-package "SCHNEIDER_103"))
    (make-package "SCHNEIDER_103"))
  (shadow 'Retour_cellule_103 (find-package "SCHNEIDER_103")))
(unless (find-package "SCHNEIDER_103::RETOUR_CELLULE_103")
  (make-package "SCHNEIDER_103::RETOUR_CELLULE_103"))
(unless (find-package "SCHNEIDER_103::RETOUR_CELLULE_103REQUEST")
  (make-package "SCHNEIDER_103::RETOUR_CELLULE_103REQUEST"))
(unless (find-package "SCHNEIDER_103::RETOUR_CELLULE_103RESPONSE")
  (make-package "SCHNEIDER_103::RETOUR_CELLULE_103RESPONSE"))

(in-package "ROS")





(defclass schneider_103::Retour_cellule_103Request
  :super ros::object
  :slots (_memoire ))

(defmethod schneider_103::Retour_cellule_103Request
  (:init
   (&key
    ((:memoire __memoire) 0)
    )
   (send-super :init)
   (setq _memoire (round __memoire))
   self)
  (:memoire
   (&optional __memoire)
   (if __memoire (setq _memoire __memoire)) _memoire)
  (:serialization-length
   ()
   (+
    ;; int32 _memoire
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _memoire
       (write-long _memoire s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _memoire
     (setq _memoire (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass schneider_103::Retour_cellule_103Response
  :super ros::object
  :slots (_ST6 _ST7 _ST18 _ST19 _R3D _R4D _R9D _R10D _R3G _R4G _R9G _R10G _D3 _D4 _D9 _D10 _V3 _V4 _V9 _V10 _PS7 _PS18 _PS19 _D3D _D4D _D9D _D10D _D3G _D4G _D9G _D10G _CP3 _CP8 ))

(defmethod schneider_103::Retour_cellule_103Response
  (:init
   (&key
    ((:ST6 __ST6) nil)
    ((:ST7 __ST7) nil)
    ((:ST18 __ST18) nil)
    ((:ST19 __ST19) nil)
    ((:R3D __R3D) nil)
    ((:R4D __R4D) nil)
    ((:R9D __R9D) nil)
    ((:R10D __R10D) nil)
    ((:R3G __R3G) nil)
    ((:R4G __R4G) nil)
    ((:R9G __R9G) nil)
    ((:R10G __R10G) nil)
    ((:D3 __D3) nil)
    ((:D4 __D4) nil)
    ((:D9 __D9) nil)
    ((:D10 __D10) nil)
    ((:V3 __V3) nil)
    ((:V4 __V4) nil)
    ((:V9 __V9) nil)
    ((:V10 __V10) nil)
    ((:PS7 __PS7) nil)
    ((:PS18 __PS18) nil)
    ((:PS19 __PS19) nil)
    ((:D3D __D3D) nil)
    ((:D4D __D4D) nil)
    ((:D9D __D9D) nil)
    ((:D10D __D10D) nil)
    ((:D3G __D3G) nil)
    ((:D4G __D4G) nil)
    ((:D9G __D9G) nil)
    ((:D10G __D10G) nil)
    ((:CP3 __CP3) nil)
    ((:CP8 __CP8) nil)
    )
   (send-super :init)
   (setq _ST6 __ST6)
   (setq _ST7 __ST7)
   (setq _ST18 __ST18)
   (setq _ST19 __ST19)
   (setq _R3D __R3D)
   (setq _R4D __R4D)
   (setq _R9D __R9D)
   (setq _R10D __R10D)
   (setq _R3G __R3G)
   (setq _R4G __R4G)
   (setq _R9G __R9G)
   (setq _R10G __R10G)
   (setq _D3 __D3)
   (setq _D4 __D4)
   (setq _D9 __D9)
   (setq _D10 __D10)
   (setq _V3 __V3)
   (setq _V4 __V4)
   (setq _V9 __V9)
   (setq _V10 __V10)
   (setq _PS7 __PS7)
   (setq _PS18 __PS18)
   (setq _PS19 __PS19)
   (setq _D3D __D3D)
   (setq _D4D __D4D)
   (setq _D9D __D9D)
   (setq _D10D __D10D)
   (setq _D3G __D3G)
   (setq _D4G __D4G)
   (setq _D9G __D9G)
   (setq _D10G __D10G)
   (setq _CP3 __CP3)
   (setq _CP8 __CP8)
   self)
  (:ST6
   (&optional (__ST6 :null))
   (if (not (eq __ST6 :null)) (setq _ST6 __ST6)) _ST6)
  (:ST7
   (&optional (__ST7 :null))
   (if (not (eq __ST7 :null)) (setq _ST7 __ST7)) _ST7)
  (:ST18
   (&optional (__ST18 :null))
   (if (not (eq __ST18 :null)) (setq _ST18 __ST18)) _ST18)
  (:ST19
   (&optional (__ST19 :null))
   (if (not (eq __ST19 :null)) (setq _ST19 __ST19)) _ST19)
  (:R3D
   (&optional (__R3D :null))
   (if (not (eq __R3D :null)) (setq _R3D __R3D)) _R3D)
  (:R4D
   (&optional (__R4D :null))
   (if (not (eq __R4D :null)) (setq _R4D __R4D)) _R4D)
  (:R9D
   (&optional (__R9D :null))
   (if (not (eq __R9D :null)) (setq _R9D __R9D)) _R9D)
  (:R10D
   (&optional (__R10D :null))
   (if (not (eq __R10D :null)) (setq _R10D __R10D)) _R10D)
  (:R3G
   (&optional (__R3G :null))
   (if (not (eq __R3G :null)) (setq _R3G __R3G)) _R3G)
  (:R4G
   (&optional (__R4G :null))
   (if (not (eq __R4G :null)) (setq _R4G __R4G)) _R4G)
  (:R9G
   (&optional (__R9G :null))
   (if (not (eq __R9G :null)) (setq _R9G __R9G)) _R9G)
  (:R10G
   (&optional (__R10G :null))
   (if (not (eq __R10G :null)) (setq _R10G __R10G)) _R10G)
  (:D3
   (&optional (__D3 :null))
   (if (not (eq __D3 :null)) (setq _D3 __D3)) _D3)
  (:D4
   (&optional (__D4 :null))
   (if (not (eq __D4 :null)) (setq _D4 __D4)) _D4)
  (:D9
   (&optional (__D9 :null))
   (if (not (eq __D9 :null)) (setq _D9 __D9)) _D9)
  (:D10
   (&optional (__D10 :null))
   (if (not (eq __D10 :null)) (setq _D10 __D10)) _D10)
  (:V3
   (&optional (__V3 :null))
   (if (not (eq __V3 :null)) (setq _V3 __V3)) _V3)
  (:V4
   (&optional (__V4 :null))
   (if (not (eq __V4 :null)) (setq _V4 __V4)) _V4)
  (:V9
   (&optional (__V9 :null))
   (if (not (eq __V9 :null)) (setq _V9 __V9)) _V9)
  (:V10
   (&optional (__V10 :null))
   (if (not (eq __V10 :null)) (setq _V10 __V10)) _V10)
  (:PS7
   (&optional (__PS7 :null))
   (if (not (eq __PS7 :null)) (setq _PS7 __PS7)) _PS7)
  (:PS18
   (&optional (__PS18 :null))
   (if (not (eq __PS18 :null)) (setq _PS18 __PS18)) _PS18)
  (:PS19
   (&optional (__PS19 :null))
   (if (not (eq __PS19 :null)) (setq _PS19 __PS19)) _PS19)
  (:D3D
   (&optional (__D3D :null))
   (if (not (eq __D3D :null)) (setq _D3D __D3D)) _D3D)
  (:D4D
   (&optional (__D4D :null))
   (if (not (eq __D4D :null)) (setq _D4D __D4D)) _D4D)
  (:D9D
   (&optional (__D9D :null))
   (if (not (eq __D9D :null)) (setq _D9D __D9D)) _D9D)
  (:D10D
   (&optional (__D10D :null))
   (if (not (eq __D10D :null)) (setq _D10D __D10D)) _D10D)
  (:D3G
   (&optional (__D3G :null))
   (if (not (eq __D3G :null)) (setq _D3G __D3G)) _D3G)
  (:D4G
   (&optional (__D4G :null))
   (if (not (eq __D4G :null)) (setq _D4G __D4G)) _D4G)
  (:D9G
   (&optional (__D9G :null))
   (if (not (eq __D9G :null)) (setq _D9G __D9G)) _D9G)
  (:D10G
   (&optional (__D10G :null))
   (if (not (eq __D10G :null)) (setq _D10G __D10G)) _D10G)
  (:CP3
   (&optional (__CP3 :null))
   (if (not (eq __CP3 :null)) (setq _CP3 __CP3)) _CP3)
  (:CP8
   (&optional (__CP8 :null))
   (if (not (eq __CP8 :null)) (setq _CP8 __CP8)) _CP8)
  (:serialization-length
   ()
   (+
    ;; bool _ST6
    1
    ;; bool _ST7
    1
    ;; bool _ST18
    1
    ;; bool _ST19
    1
    ;; bool _R3D
    1
    ;; bool _R4D
    1
    ;; bool _R9D
    1
    ;; bool _R10D
    1
    ;; bool _R3G
    1
    ;; bool _R4G
    1
    ;; bool _R9G
    1
    ;; bool _R10G
    1
    ;; bool _D3
    1
    ;; bool _D4
    1
    ;; bool _D9
    1
    ;; bool _D10
    1
    ;; bool _V3
    1
    ;; bool _V4
    1
    ;; bool _V9
    1
    ;; bool _V10
    1
    ;; bool _PS7
    1
    ;; bool _PS18
    1
    ;; bool _PS19
    1
    ;; bool _D3D
    1
    ;; bool _D4D
    1
    ;; bool _D9D
    1
    ;; bool _D10D
    1
    ;; bool _D3G
    1
    ;; bool _D4G
    1
    ;; bool _D9G
    1
    ;; bool _D10G
    1
    ;; bool _CP3
    1
    ;; bool _CP8
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ST6
       (if _ST6 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST7
       (if _ST7 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST18
       (if _ST18 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST19
       (if _ST19 (write-byte -1 s) (write-byte 0 s))
     ;; bool _R3D
       (if _R3D (write-byte -1 s) (write-byte 0 s))
     ;; bool _R4D
       (if _R4D (write-byte -1 s) (write-byte 0 s))
     ;; bool _R9D
       (if _R9D (write-byte -1 s) (write-byte 0 s))
     ;; bool _R10D
       (if _R10D (write-byte -1 s) (write-byte 0 s))
     ;; bool _R3G
       (if _R3G (write-byte -1 s) (write-byte 0 s))
     ;; bool _R4G
       (if _R4G (write-byte -1 s) (write-byte 0 s))
     ;; bool _R9G
       (if _R9G (write-byte -1 s) (write-byte 0 s))
     ;; bool _R10G
       (if _R10G (write-byte -1 s) (write-byte 0 s))
     ;; bool _D3
       (if _D3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _D4
       (if _D4 (write-byte -1 s) (write-byte 0 s))
     ;; bool _D9
       (if _D9 (write-byte -1 s) (write-byte 0 s))
     ;; bool _D10
       (if _D10 (write-byte -1 s) (write-byte 0 s))
     ;; bool _V3
       (if _V3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _V4
       (if _V4 (write-byte -1 s) (write-byte 0 s))
     ;; bool _V9
       (if _V9 (write-byte -1 s) (write-byte 0 s))
     ;; bool _V10
       (if _V10 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS7
       (if _PS7 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS18
       (if _PS18 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS19
       (if _PS19 (write-byte -1 s) (write-byte 0 s))
     ;; bool _D3D
       (if _D3D (write-byte -1 s) (write-byte 0 s))
     ;; bool _D4D
       (if _D4D (write-byte -1 s) (write-byte 0 s))
     ;; bool _D9D
       (if _D9D (write-byte -1 s) (write-byte 0 s))
     ;; bool _D10D
       (if _D10D (write-byte -1 s) (write-byte 0 s))
     ;; bool _D3G
       (if _D3G (write-byte -1 s) (write-byte 0 s))
     ;; bool _D4G
       (if _D4G (write-byte -1 s) (write-byte 0 s))
     ;; bool _D9G
       (if _D9G (write-byte -1 s) (write-byte 0 s))
     ;; bool _D10G
       (if _D10G (write-byte -1 s) (write-byte 0 s))
     ;; bool _CP3
       (if _CP3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _CP8
       (if _CP8 (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ST6
     (setq _ST6 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST7
     (setq _ST7 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST18
     (setq _ST18 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST19
     (setq _ST19 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R3D
     (setq _R3D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R4D
     (setq _R4D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R9D
     (setq _R9D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R10D
     (setq _R10D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R3G
     (setq _R3G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R4G
     (setq _R4G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R9G
     (setq _R9G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R10G
     (setq _R10G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D3
     (setq _D3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D4
     (setq _D4 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D9
     (setq _D9 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D10
     (setq _D10 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _V3
     (setq _V3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _V4
     (setq _V4 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _V9
     (setq _V9 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _V10
     (setq _V10 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS7
     (setq _PS7 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS18
     (setq _PS18 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS19
     (setq _PS19 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D3D
     (setq _D3D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D4D
     (setq _D4D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D9D
     (setq _D9D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D10D
     (setq _D10D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D3G
     (setq _D3G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D4G
     (setq _D4G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D9G
     (setq _D9G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D10G
     (setq _D10G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CP3
     (setq _CP3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CP8
     (setq _CP8 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass schneider_103::Retour_cellule_103
  :super ros::object
  :slots ())

(setf (get schneider_103::Retour_cellule_103 :md5sum-) "144215a6005e994c4be2aa0a861077f5")
(setf (get schneider_103::Retour_cellule_103 :datatype-) "schneider_103/Retour_cellule_103")
(setf (get schneider_103::Retour_cellule_103 :request) schneider_103::Retour_cellule_103Request)
(setf (get schneider_103::Retour_cellule_103 :response) schneider_103::Retour_cellule_103Response)

(defmethod schneider_103::Retour_cellule_103Request
  (:response () (instance schneider_103::Retour_cellule_103Response :init)))

(setf (get schneider_103::Retour_cellule_103Request :md5sum-) "144215a6005e994c4be2aa0a861077f5")
(setf (get schneider_103::Retour_cellule_103Request :datatype-) "schneider_103/Retour_cellule_103Request")
(setf (get schneider_103::Retour_cellule_103Request :definition-)
      "int32 memoire
---

bool ST6
bool ST7
bool ST18
bool ST19
bool R3D
bool R4D
bool R9D
bool R10D
bool R3G
bool R4G
bool R9G
bool R10G
bool D3
bool D4
bool D9
bool D10
bool V3
bool V4
bool V9
bool V10
bool PS7
bool PS18
bool PS19
bool D3D
bool D4D
bool D9D
bool D10D
bool D3G
bool D4G
bool D9G
bool D10G
bool CP3
bool CP8

")

(setf (get schneider_103::Retour_cellule_103Response :md5sum-) "144215a6005e994c4be2aa0a861077f5")
(setf (get schneider_103::Retour_cellule_103Response :datatype-) "schneider_103/Retour_cellule_103Response")
(setf (get schneider_103::Retour_cellule_103Response :definition-)
      "int32 memoire
---

bool ST6
bool ST7
bool ST18
bool ST19
bool R3D
bool R4D
bool R9D
bool R10D
bool R3G
bool R4G
bool R9G
bool R10G
bool D3
bool D4
bool D9
bool D10
bool V3
bool V4
bool V9
bool V10
bool PS7
bool PS18
bool PS19
bool D3D
bool D4D
bool D9D
bool D10D
bool D3G
bool D4G
bool D9G
bool D10G
bool CP3
bool CP8

")



(provide :schneider_103/Retour_cellule_103 "144215a6005e994c4be2aa0a861077f5")


