
(cl:in-package :asdf)

(defsystem "robots-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FaireTacheMsg" :depends-on ("_package_FaireTacheMsg"))
    (:file "_package_FaireTacheMsg" :depends-on ("_package"))
    (:file "MoveRobot" :depends-on ("_package_MoveRobot"))
    (:file "_package_MoveRobot" :depends-on ("_package"))
    (:file "Msg_numrobot" :depends-on ("_package_Msg_numrobot"))
    (:file "_package_Msg_numrobot" :depends-on ("_package"))
  ))