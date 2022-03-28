;; Auto-generated. Do not edit!


(when (boundp 'schneider_104::Retour_cellule_104)
  (if (not (find-package "SCHNEIDER_104"))
    (make-package "SCHNEIDER_104"))
  (shadow 'Retour_cellule_104 (find-package "SCHNEIDER_104")))
(unless (find-package "SCHNEIDER_104::RETOUR_CELLULE_104")
  (make-package "SCHNEIDER_104::RETOUR_CELLULE_104"))
(unless (find-package "SCHNEIDER_104::RETOUR_CELLULE_104REQUEST")
  (make-package "SCHNEIDER_104::RETOUR_CELLULE_104REQUEST"))
(unless (find-package "SCHNEIDER_104::RETOUR_CELLULE_104RESPONSE")
  (make-package "SCHNEIDER_104::RETOUR_CELLULE_104RESPONSE"))

(in-package "ROS")





(defclass schneider_104::Retour_cellule_104Request
  :super ros::object
  :slots (_memoire ))

(defmethod schneider_104::Retour_cellule_104Request
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

(defclass schneider_104::Retour_cellule_104Response
  :super ros::object
  :slots (_ST8 _ST9 _ST10 _ST11 _ST12 _ST13 _ST14 _ST15 _ST16 _ST17 _R5D _R6D _R7D _R8D _R5G _R6G _R7G _R8G _PI3 _PI4 _PI5 _PI6 _D5 _D6 _D7 _D8 _V5 _V6 _V7 _V8 _PS8 _PS9 _PS10 _PS11 _PS12 _PS13 _PS14 _PS15 _PS16 _PS17 _D5D _D6D _D7D _D8D _D5G _D6G _D7G _D8G _CPI3 _CPI4 _CPI5 _CPI6 _CP4 _CP5 _CP6 _CP7 ))

(defmethod schneider_104::Retour_cellule_104Response
  (:init
   (&key
    ((:ST8 __ST8) nil)
    ((:ST9 __ST9) nil)
    ((:ST10 __ST10) nil)
    ((:ST11 __ST11) nil)
    ((:ST12 __ST12) nil)
    ((:ST13 __ST13) nil)
    ((:ST14 __ST14) nil)
    ((:ST15 __ST15) nil)
    ((:ST16 __ST16) nil)
    ((:ST17 __ST17) nil)
    ((:R5D __R5D) nil)
    ((:R6D __R6D) nil)
    ((:R7D __R7D) nil)
    ((:R8D __R8D) nil)
    ((:R5G __R5G) nil)
    ((:R6G __R6G) nil)
    ((:R7G __R7G) nil)
    ((:R8G __R8G) nil)
    ((:PI3 __PI3) nil)
    ((:PI4 __PI4) nil)
    ((:PI5 __PI5) nil)
    ((:PI6 __PI6) nil)
    ((:D5 __D5) nil)
    ((:D6 __D6) nil)
    ((:D7 __D7) nil)
    ((:D8 __D8) nil)
    ((:V5 __V5) nil)
    ((:V6 __V6) nil)
    ((:V7 __V7) nil)
    ((:V8 __V8) nil)
    ((:PS8 __PS8) nil)
    ((:PS9 __PS9) nil)
    ((:PS10 __PS10) nil)
    ((:PS11 __PS11) nil)
    ((:PS12 __PS12) nil)
    ((:PS13 __PS13) nil)
    ((:PS14 __PS14) nil)
    ((:PS15 __PS15) nil)
    ((:PS16 __PS16) nil)
    ((:PS17 __PS17) nil)
    ((:D5D __D5D) nil)
    ((:D6D __D6D) nil)
    ((:D7D __D7D) nil)
    ((:D8D __D8D) nil)
    ((:D5G __D5G) nil)
    ((:D6G __D6G) nil)
    ((:D7G __D7G) nil)
    ((:D8G __D8G) nil)
    ((:CPI3 __CPI3) nil)
    ((:CPI4 __CPI4) nil)
    ((:CPI5 __CPI5) nil)
    ((:CPI6 __CPI6) nil)
    ((:CP4 __CP4) nil)
    ((:CP5 __CP5) nil)
    ((:CP6 __CP6) nil)
    ((:CP7 __CP7) nil)
    )
   (send-super :init)
   (setq _ST8 __ST8)
   (setq _ST9 __ST9)
   (setq _ST10 __ST10)
   (setq _ST11 __ST11)
   (setq _ST12 __ST12)
   (setq _ST13 __ST13)
   (setq _ST14 __ST14)
   (setq _ST15 __ST15)
   (setq _ST16 __ST16)
   (setq _ST17 __ST17)
   (setq _R5D __R5D)
   (setq _R6D __R6D)
   (setq _R7D __R7D)
   (setq _R8D __R8D)
   (setq _R5G __R5G)
   (setq _R6G __R6G)
   (setq _R7G __R7G)
   (setq _R8G __R8G)
   (setq _PI3 __PI3)
   (setq _PI4 __PI4)
   (setq _PI5 __PI5)
   (setq _PI6 __PI6)
   (setq _D5 __D5)
   (setq _D6 __D6)
   (setq _D7 __D7)
   (setq _D8 __D8)
   (setq _V5 __V5)
   (setq _V6 __V6)
   (setq _V7 __V7)
   (setq _V8 __V8)
   (setq _PS8 __PS8)
   (setq _PS9 __PS9)
   (setq _PS10 __PS10)
   (setq _PS11 __PS11)
   (setq _PS12 __PS12)
   (setq _PS13 __PS13)
   (setq _PS14 __PS14)
   (setq _PS15 __PS15)
   (setq _PS16 __PS16)
   (setq _PS17 __PS17)
   (setq _D5D __D5D)
   (setq _D6D __D6D)
   (setq _D7D __D7D)
   (setq _D8D __D8D)
   (setq _D5G __D5G)
   (setq _D6G __D6G)
   (setq _D7G __D7G)
   (setq _D8G __D8G)
   (setq _CPI3 __CPI3)
   (setq _CPI4 __CPI4)
   (setq _CPI5 __CPI5)
   (setq _CPI6 __CPI6)
   (setq _CP4 __CP4)
   (setq _CP5 __CP5)
   (setq _CP6 __CP6)
   (setq _CP7 __CP7)
   self)
  (:ST8
   (&optional (__ST8 :null))
   (if (not (eq __ST8 :null)) (setq _ST8 __ST8)) _ST8)
  (:ST9
   (&optional (__ST9 :null))
   (if (not (eq __ST9 :null)) (setq _ST9 __ST9)) _ST9)
  (:ST10
   (&optional (__ST10 :null))
   (if (not (eq __ST10 :null)) (setq _ST10 __ST10)) _ST10)
  (:ST11
   (&optional (__ST11 :null))
   (if (not (eq __ST11 :null)) (setq _ST11 __ST11)) _ST11)
  (:ST12
   (&optional (__ST12 :null))
   (if (not (eq __ST12 :null)) (setq _ST12 __ST12)) _ST12)
  (:ST13
   (&optional (__ST13 :null))
   (if (not (eq __ST13 :null)) (setq _ST13 __ST13)) _ST13)
  (:ST14
   (&optional (__ST14 :null))
   (if (not (eq __ST14 :null)) (setq _ST14 __ST14)) _ST14)
  (:ST15
   (&optional (__ST15 :null))
   (if (not (eq __ST15 :null)) (setq _ST15 __ST15)) _ST15)
  (:ST16
   (&optional (__ST16 :null))
   (if (not (eq __ST16 :null)) (setq _ST16 __ST16)) _ST16)
  (:ST17
   (&optional (__ST17 :null))
   (if (not (eq __ST17 :null)) (setq _ST17 __ST17)) _ST17)
  (:R5D
   (&optional (__R5D :null))
   (if (not (eq __R5D :null)) (setq _R5D __R5D)) _R5D)
  (:R6D
   (&optional (__R6D :null))
   (if (not (eq __R6D :null)) (setq _R6D __R6D)) _R6D)
  (:R7D
   (&optional (__R7D :null))
   (if (not (eq __R7D :null)) (setq _R7D __R7D)) _R7D)
  (:R8D
   (&optional (__R8D :null))
   (if (not (eq __R8D :null)) (setq _R8D __R8D)) _R8D)
  (:R5G
   (&optional (__R5G :null))
   (if (not (eq __R5G :null)) (setq _R5G __R5G)) _R5G)
  (:R6G
   (&optional (__R6G :null))
   (if (not (eq __R6G :null)) (setq _R6G __R6G)) _R6G)
  (:R7G
   (&optional (__R7G :null))
   (if (not (eq __R7G :null)) (setq _R7G __R7G)) _R7G)
  (:R8G
   (&optional (__R8G :null))
   (if (not (eq __R8G :null)) (setq _R8G __R8G)) _R8G)
  (:PI3
   (&optional (__PI3 :null))
   (if (not (eq __PI3 :null)) (setq _PI3 __PI3)) _PI3)
  (:PI4
   (&optional (__PI4 :null))
   (if (not (eq __PI4 :null)) (setq _PI4 __PI4)) _PI4)
  (:PI5
   (&optional (__PI5 :null))
   (if (not (eq __PI5 :null)) (setq _PI5 __PI5)) _PI5)
  (:PI6
   (&optional (__PI6 :null))
   (if (not (eq __PI6 :null)) (setq _PI6 __PI6)) _PI6)
  (:D5
   (&optional (__D5 :null))
   (if (not (eq __D5 :null)) (setq _D5 __D5)) _D5)
  (:D6
   (&optional (__D6 :null))
   (if (not (eq __D6 :null)) (setq _D6 __D6)) _D6)
  (:D7
   (&optional (__D7 :null))
   (if (not (eq __D7 :null)) (setq _D7 __D7)) _D7)
  (:D8
   (&optional (__D8 :null))
   (if (not (eq __D8 :null)) (setq _D8 __D8)) _D8)
  (:V5
   (&optional (__V5 :null))
   (if (not (eq __V5 :null)) (setq _V5 __V5)) _V5)
  (:V6
   (&optional (__V6 :null))
   (if (not (eq __V6 :null)) (setq _V6 __V6)) _V6)
  (:V7
   (&optional (__V7 :null))
   (if (not (eq __V7 :null)) (setq _V7 __V7)) _V7)
  (:V8
   (&optional (__V8 :null))
   (if (not (eq __V8 :null)) (setq _V8 __V8)) _V8)
  (:PS8
   (&optional (__PS8 :null))
   (if (not (eq __PS8 :null)) (setq _PS8 __PS8)) _PS8)
  (:PS9
   (&optional (__PS9 :null))
   (if (not (eq __PS9 :null)) (setq _PS9 __PS9)) _PS9)
  (:PS10
   (&optional (__PS10 :null))
   (if (not (eq __PS10 :null)) (setq _PS10 __PS10)) _PS10)
  (:PS11
   (&optional (__PS11 :null))
   (if (not (eq __PS11 :null)) (setq _PS11 __PS11)) _PS11)
  (:PS12
   (&optional (__PS12 :null))
   (if (not (eq __PS12 :null)) (setq _PS12 __PS12)) _PS12)
  (:PS13
   (&optional (__PS13 :null))
   (if (not (eq __PS13 :null)) (setq _PS13 __PS13)) _PS13)
  (:PS14
   (&optional (__PS14 :null))
   (if (not (eq __PS14 :null)) (setq _PS14 __PS14)) _PS14)
  (:PS15
   (&optional (__PS15 :null))
   (if (not (eq __PS15 :null)) (setq _PS15 __PS15)) _PS15)
  (:PS16
   (&optional (__PS16 :null))
   (if (not (eq __PS16 :null)) (setq _PS16 __PS16)) _PS16)
  (:PS17
   (&optional (__PS17 :null))
   (if (not (eq __PS17 :null)) (setq _PS17 __PS17)) _PS17)
  (:D5D
   (&optional (__D5D :null))
   (if (not (eq __D5D :null)) (setq _D5D __D5D)) _D5D)
  (:D6D
   (&optional (__D6D :null))
   (if (not (eq __D6D :null)) (setq _D6D __D6D)) _D6D)
  (:D7D
   (&optional (__D7D :null))
   (if (not (eq __D7D :null)) (setq _D7D __D7D)) _D7D)
  (:D8D
   (&optional (__D8D :null))
   (if (not (eq __D8D :null)) (setq _D8D __D8D)) _D8D)
  (:D5G
   (&optional (__D5G :null))
   (if (not (eq __D5G :null)) (setq _D5G __D5G)) _D5G)
  (:D6G
   (&optional (__D6G :null))
   (if (not (eq __D6G :null)) (setq _D6G __D6G)) _D6G)
  (:D7G
   (&optional (__D7G :null))
   (if (not (eq __D7G :null)) (setq _D7G __D7G)) _D7G)
  (:D8G
   (&optional (__D8G :null))
   (if (not (eq __D8G :null)) (setq _D8G __D8G)) _D8G)
  (:CPI3
   (&optional (__CPI3 :null))
   (if (not (eq __CPI3 :null)) (setq _CPI3 __CPI3)) _CPI3)
  (:CPI4
   (&optional (__CPI4 :null))
   (if (not (eq __CPI4 :null)) (setq _CPI4 __CPI4)) _CPI4)
  (:CPI5
   (&optional (__CPI5 :null))
   (if (not (eq __CPI5 :null)) (setq _CPI5 __CPI5)) _CPI5)
  (:CPI6
   (&optional (__CPI6 :null))
   (if (not (eq __CPI6 :null)) (setq _CPI6 __CPI6)) _CPI6)
  (:CP4
   (&optional (__CP4 :null))
   (if (not (eq __CP4 :null)) (setq _CP4 __CP4)) _CP4)
  (:CP5
   (&optional (__CP5 :null))
   (if (not (eq __CP5 :null)) (setq _CP5 __CP5)) _CP5)
  (:CP6
   (&optional (__CP6 :null))
   (if (not (eq __CP6 :null)) (setq _CP6 __CP6)) _CP6)
  (:CP7
   (&optional (__CP7 :null))
   (if (not (eq __CP7 :null)) (setq _CP7 __CP7)) _CP7)
  (:serialization-length
   ()
   (+
    ;; bool _ST8
    1
    ;; bool _ST9
    1
    ;; bool _ST10
    1
    ;; bool _ST11
    1
    ;; bool _ST12
    1
    ;; bool _ST13
    1
    ;; bool _ST14
    1
    ;; bool _ST15
    1
    ;; bool _ST16
    1
    ;; bool _ST17
    1
    ;; bool _R5D
    1
    ;; bool _R6D
    1
    ;; bool _R7D
    1
    ;; bool _R8D
    1
    ;; bool _R5G
    1
    ;; bool _R6G
    1
    ;; bool _R7G
    1
    ;; bool _R8G
    1
    ;; bool _PI3
    1
    ;; bool _PI4
    1
    ;; bool _PI5
    1
    ;; bool _PI6
    1
    ;; bool _D5
    1
    ;; bool _D6
    1
    ;; bool _D7
    1
    ;; bool _D8
    1
    ;; bool _V5
    1
    ;; bool _V6
    1
    ;; bool _V7
    1
    ;; bool _V8
    1
    ;; bool _PS8
    1
    ;; bool _PS9
    1
    ;; bool _PS10
    1
    ;; bool _PS11
    1
    ;; bool _PS12
    1
    ;; bool _PS13
    1
    ;; bool _PS14
    1
    ;; bool _PS15
    1
    ;; bool _PS16
    1
    ;; bool _PS17
    1
    ;; bool _D5D
    1
    ;; bool _D6D
    1
    ;; bool _D7D
    1
    ;; bool _D8D
    1
    ;; bool _D5G
    1
    ;; bool _D6G
    1
    ;; bool _D7G
    1
    ;; bool _D8G
    1
    ;; bool _CPI3
    1
    ;; bool _CPI4
    1
    ;; bool _CPI5
    1
    ;; bool _CPI6
    1
    ;; bool _CP4
    1
    ;; bool _CP5
    1
    ;; bool _CP6
    1
    ;; bool _CP7
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ST8
       (if _ST8 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST9
       (if _ST9 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST10
       (if _ST10 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST11
       (if _ST11 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST12
       (if _ST12 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST13
       (if _ST13 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST14
       (if _ST14 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST15
       (if _ST15 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST16
       (if _ST16 (write-byte -1 s) (write-byte 0 s))
     ;; bool _ST17
       (if _ST17 (write-byte -1 s) (write-byte 0 s))
     ;; bool _R5D
       (if _R5D (write-byte -1 s) (write-byte 0 s))
     ;; bool _R6D
       (if _R6D (write-byte -1 s) (write-byte 0 s))
     ;; bool _R7D
       (if _R7D (write-byte -1 s) (write-byte 0 s))
     ;; bool _R8D
       (if _R8D (write-byte -1 s) (write-byte 0 s))
     ;; bool _R5G
       (if _R5G (write-byte -1 s) (write-byte 0 s))
     ;; bool _R6G
       (if _R6G (write-byte -1 s) (write-byte 0 s))
     ;; bool _R7G
       (if _R7G (write-byte -1 s) (write-byte 0 s))
     ;; bool _R8G
       (if _R8G (write-byte -1 s) (write-byte 0 s))
     ;; bool _PI3
       (if _PI3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PI4
       (if _PI4 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PI5
       (if _PI5 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PI6
       (if _PI6 (write-byte -1 s) (write-byte 0 s))
     ;; bool _D5
       (if _D5 (write-byte -1 s) (write-byte 0 s))
     ;; bool _D6
       (if _D6 (write-byte -1 s) (write-byte 0 s))
     ;; bool _D7
       (if _D7 (write-byte -1 s) (write-byte 0 s))
     ;; bool _D8
       (if _D8 (write-byte -1 s) (write-byte 0 s))
     ;; bool _V5
       (if _V5 (write-byte -1 s) (write-byte 0 s))
     ;; bool _V6
       (if _V6 (write-byte -1 s) (write-byte 0 s))
     ;; bool _V7
       (if _V7 (write-byte -1 s) (write-byte 0 s))
     ;; bool _V8
       (if _V8 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS8
       (if _PS8 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS9
       (if _PS9 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS10
       (if _PS10 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS11
       (if _PS11 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS12
       (if _PS12 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS13
       (if _PS13 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS14
       (if _PS14 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS15
       (if _PS15 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS16
       (if _PS16 (write-byte -1 s) (write-byte 0 s))
     ;; bool _PS17
       (if _PS17 (write-byte -1 s) (write-byte 0 s))
     ;; bool _D5D
       (if _D5D (write-byte -1 s) (write-byte 0 s))
     ;; bool _D6D
       (if _D6D (write-byte -1 s) (write-byte 0 s))
     ;; bool _D7D
       (if _D7D (write-byte -1 s) (write-byte 0 s))
     ;; bool _D8D
       (if _D8D (write-byte -1 s) (write-byte 0 s))
     ;; bool _D5G
       (if _D5G (write-byte -1 s) (write-byte 0 s))
     ;; bool _D6G
       (if _D6G (write-byte -1 s) (write-byte 0 s))
     ;; bool _D7G
       (if _D7G (write-byte -1 s) (write-byte 0 s))
     ;; bool _D8G
       (if _D8G (write-byte -1 s) (write-byte 0 s))
     ;; bool _CPI3
       (if _CPI3 (write-byte -1 s) (write-byte 0 s))
     ;; bool _CPI4
       (if _CPI4 (write-byte -1 s) (write-byte 0 s))
     ;; bool _CPI5
       (if _CPI5 (write-byte -1 s) (write-byte 0 s))
     ;; bool _CPI6
       (if _CPI6 (write-byte -1 s) (write-byte 0 s))
     ;; bool _CP4
       (if _CP4 (write-byte -1 s) (write-byte 0 s))
     ;; bool _CP5
       (if _CP5 (write-byte -1 s) (write-byte 0 s))
     ;; bool _CP6
       (if _CP6 (write-byte -1 s) (write-byte 0 s))
     ;; bool _CP7
       (if _CP7 (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ST8
     (setq _ST8 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST9
     (setq _ST9 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST10
     (setq _ST10 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST11
     (setq _ST11 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST12
     (setq _ST12 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST13
     (setq _ST13 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST14
     (setq _ST14 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST15
     (setq _ST15 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST16
     (setq _ST16 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ST17
     (setq _ST17 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R5D
     (setq _R5D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R6D
     (setq _R6D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R7D
     (setq _R7D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R8D
     (setq _R8D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R5G
     (setq _R5G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R6G
     (setq _R6G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R7G
     (setq _R7G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _R8G
     (setq _R8G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PI3
     (setq _PI3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PI4
     (setq _PI4 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PI5
     (setq _PI5 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PI6
     (setq _PI6 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D5
     (setq _D5 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D6
     (setq _D6 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D7
     (setq _D7 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D8
     (setq _D8 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _V5
     (setq _V5 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _V6
     (setq _V6 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _V7
     (setq _V7 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _V8
     (setq _V8 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS8
     (setq _PS8 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS9
     (setq _PS9 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS10
     (setq _PS10 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS11
     (setq _PS11 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS12
     (setq _PS12 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS13
     (setq _PS13 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS14
     (setq _PS14 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS15
     (setq _PS15 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS16
     (setq _PS16 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _PS17
     (setq _PS17 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D5D
     (setq _D5D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D6D
     (setq _D6D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D7D
     (setq _D7D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D8D
     (setq _D8D (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D5G
     (setq _D5G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D6G
     (setq _D6G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D7G
     (setq _D7G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _D8G
     (setq _D8G (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CPI3
     (setq _CPI3 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CPI4
     (setq _CPI4 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CPI5
     (setq _CPI5 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CPI6
     (setq _CPI6 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CP4
     (setq _CP4 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CP5
     (setq _CP5 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CP6
     (setq _CP6 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _CP7
     (setq _CP7 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass schneider_104::Retour_cellule_104
  :super ros::object
  :slots ())

(setf (get schneider_104::Retour_cellule_104 :md5sum-) "da2dff768924005c2c1434c86e6e1a14")
(setf (get schneider_104::Retour_cellule_104 :datatype-) "schneider_104/Retour_cellule_104")
(setf (get schneider_104::Retour_cellule_104 :request) schneider_104::Retour_cellule_104Request)
(setf (get schneider_104::Retour_cellule_104 :response) schneider_104::Retour_cellule_104Response)

(defmethod schneider_104::Retour_cellule_104Request
  (:response () (instance schneider_104::Retour_cellule_104Response :init)))

(setf (get schneider_104::Retour_cellule_104Request :md5sum-) "da2dff768924005c2c1434c86e6e1a14")
(setf (get schneider_104::Retour_cellule_104Request :datatype-) "schneider_104/Retour_cellule_104Request")
(setf (get schneider_104::Retour_cellule_104Request :definition-)
      "int32 memoire
---
bool ST8
bool ST9
bool ST10
bool ST11
bool ST12
bool ST13
bool ST14
bool ST15
bool ST16
bool ST17
bool R5D
bool R6D
bool R7D
bool R8D
bool R5G
bool R6G
bool R7G
bool R8G
bool PI3
bool PI4
bool PI5
bool PI6
bool D5
bool D6
bool D7
bool D8
bool V5
bool V6
bool V7
bool V8
bool PS8
bool PS9
bool PS10
bool PS11
bool PS12
bool PS13
bool PS14
bool PS15
bool PS16
bool PS17
bool D5D
bool D6D
bool D7D
bool D8D
bool D5G
bool D6G
bool D7G
bool D8G
bool CPI3
bool CPI4
bool CPI5
bool CPI6
bool CP4
bool CP5
bool CP6
bool CP7

")

(setf (get schneider_104::Retour_cellule_104Response :md5sum-) "da2dff768924005c2c1434c86e6e1a14")
(setf (get schneider_104::Retour_cellule_104Response :datatype-) "schneider_104/Retour_cellule_104Response")
(setf (get schneider_104::Retour_cellule_104Response :definition-)
      "int32 memoire
---
bool ST8
bool ST9
bool ST10
bool ST11
bool ST12
bool ST13
bool ST14
bool ST15
bool ST16
bool ST17
bool R5D
bool R6D
bool R7D
bool R8D
bool R5G
bool R6G
bool R7G
bool R8G
bool PI3
bool PI4
bool PI5
bool PI6
bool D5
bool D6
bool D7
bool D8
bool V5
bool V6
bool V7
bool V8
bool PS8
bool PS9
bool PS10
bool PS11
bool PS12
bool PS13
bool PS14
bool PS15
bool PS16
bool PS17
bool D5D
bool D6D
bool D7D
bool D8D
bool D5G
bool D6G
bool D7G
bool D8G
bool CPI3
bool CPI4
bool CPI5
bool CPI6
bool CP4
bool CP5
bool CP6
bool CP7

")



(provide :schneider_104/Retour_cellule_104 "da2dff768924005c2c1434c86e6e1a14")


