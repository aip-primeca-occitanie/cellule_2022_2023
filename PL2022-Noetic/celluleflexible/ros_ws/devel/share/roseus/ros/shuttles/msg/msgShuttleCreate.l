;; Auto-generated. Do not edit!


(when (boundp 'shuttles::msgShuttleCreate)
  (if (not (find-package "SHUTTLES"))
    (make-package "SHUTTLES"))
  (shadow 'msgShuttleCreate (find-package "SHUTTLES")))
(unless (find-package "SHUTTLES::MSGSHUTTLECREATE")
  (make-package "SHUTTLES::MSGSHUTTLECREATE"))

(in-package "ROS")
;;//! \htmlinclude msgShuttleCreate.msg.html


(defclass shuttles::msgShuttleCreate
  :super ros::object
  :slots (_name _destination _product _handle _handlePlatform _zone ))

(defmethod shuttles::msgShuttleCreate
  (:init
   (&key
    ((:name __name) "")
    ((:destination __destination) 0)
    ((:product __product) 0)
    ((:handle __handle) 0)
    ((:handlePlatform __handlePlatform) 0)
    ((:zone __zone) 0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _destination (round __destination))
   (setq _product (round __product))
   (setq _handle (round __handle))
   (setq _handlePlatform (round __handlePlatform))
   (setq _zone (round __zone))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:destination
   (&optional __destination)
   (if __destination (setq _destination __destination)) _destination)
  (:product
   (&optional __product)
   (if __product (setq _product __product)) _product)
  (:handle
   (&optional __handle)
   (if __handle (setq _handle __handle)) _handle)
  (:handlePlatform
   (&optional __handlePlatform)
   (if __handlePlatform (setq _handlePlatform __handlePlatform)) _handlePlatform)
  (:zone
   (&optional __zone)
   (if __zone (setq _zone __zone)) _zone)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; int32 _destination
    4
    ;; int32 _product
    4
    ;; int32 _handle
    4
    ;; int32 _handlePlatform
    4
    ;; int32 _zone
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; int32 _destination
       (write-long _destination s)
     ;; int32 _product
       (write-long _product s)
     ;; int32 _handle
       (write-long _handle s)
     ;; int32 _handlePlatform
       (write-long _handlePlatform s)
     ;; int32 _zone
       (write-long _zone s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _destination
     (setq _destination (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _product
     (setq _product (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _handle
     (setq _handle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _handlePlatform
     (setq _handlePlatform (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _zone
     (setq _zone (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get shuttles::msgShuttleCreate :md5sum-) "bbe2beba2a1f997c0e8c09a3d7eb1f7a")
(setf (get shuttles::msgShuttleCreate :datatype-) "shuttles/msgShuttleCreate")
(setf (get shuttles::msgShuttleCreate :definition-)
      "string name
int32 destination
int32 product
int32 handle
int32 handlePlatform
int32 zone

")



(provide :shuttles/msgShuttleCreate "bbe2beba2a1f997c0e8c09a3d7eb1f7a")


