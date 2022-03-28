;; Auto-generated. Do not edit!


(when (boundp 'shuttles::shuttle_id)
  (if (not (find-package "SHUTTLES"))
    (make-package "SHUTTLES"))
  (shadow 'shuttle_id (find-package "SHUTTLES")))
(unless (find-package "SHUTTLES::SHUTTLE_ID")
  (make-package "SHUTTLES::SHUTTLE_ID"))
(unless (find-package "SHUTTLES::SHUTTLE_IDREQUEST")
  (make-package "SHUTTLES::SHUTTLE_IDREQUEST"))
(unless (find-package "SHUTTLES::SHUTTLE_IDRESPONSE")
  (make-package "SHUTTLES::SHUTTLE_IDRESPONSE"))

(in-package "ROS")





(defclass shuttles::shuttle_idRequest
  :super ros::object
  :slots (_robot _position ))

(defmethod shuttles::shuttle_idRequest
  (:init
   (&key
    ((:robot __robot) 0)
    ((:position __position) 0)
    )
   (send-super :init)
   (setq _robot (round __robot))
   (setq _position (round __position))
   self)
  (:robot
   (&optional __robot)
   (if __robot (setq _robot __robot)) _robot)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; int32 _robot
    4
    ;; int32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _robot
       (write-long _robot s)
     ;; int32 _position
       (write-long _position s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _robot
     (setq _robot (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass shuttles::shuttle_idResponse
  :super ros::object
  :slots (_IdShuttle ))

(defmethod shuttles::shuttle_idResponse
  (:init
   (&key
    ((:IdShuttle __IdShuttle) 0)
    )
   (send-super :init)
   (setq _IdShuttle (round __IdShuttle))
   self)
  (:IdShuttle
   (&optional __IdShuttle)
   (if __IdShuttle (setq _IdShuttle __IdShuttle)) _IdShuttle)
  (:serialization-length
   ()
   (+
    ;; int32 _IdShuttle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _IdShuttle
       (write-long _IdShuttle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _IdShuttle
     (setq _IdShuttle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass shuttles::shuttle_id
  :super ros::object
  :slots ())

(setf (get shuttles::shuttle_id :md5sum-) "4ce93559e67d465c0b309aaf61a7e04b")
(setf (get shuttles::shuttle_id :datatype-) "shuttles/shuttle_id")
(setf (get shuttles::shuttle_id :request) shuttles::shuttle_idRequest)
(setf (get shuttles::shuttle_id :response) shuttles::shuttle_idResponse)

(defmethod shuttles::shuttle_idRequest
  (:response () (instance shuttles::shuttle_idResponse :init)))

(setf (get shuttles::shuttle_idRequest :md5sum-) "4ce93559e67d465c0b309aaf61a7e04b")
(setf (get shuttles::shuttle_idRequest :datatype-) "shuttles/shuttle_idRequest")
(setf (get shuttles::shuttle_idRequest :definition-)
      "int32 robot
int32 position
---
int32 IdShuttle

")

(setf (get shuttles::shuttle_idResponse :md5sum-) "4ce93559e67d465c0b309aaf61a7e04b")
(setf (get shuttles::shuttle_idResponse :datatype-) "shuttles/shuttle_idResponse")
(setf (get shuttles::shuttle_idResponse :definition-)
      "int32 robot
int32 position
---
int32 IdShuttle

")



(provide :shuttles/shuttle_id "4ce93559e67d465c0b309aaf61a7e04b")


