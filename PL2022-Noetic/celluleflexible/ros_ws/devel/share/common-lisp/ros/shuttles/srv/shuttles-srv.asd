
(cl:in-package :asdf)

(defsystem "shuttles-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "shuttle_id" :depends-on ("_package_shuttle_id"))
    (:file "_package_shuttle_id" :depends-on ("_package"))
  ))