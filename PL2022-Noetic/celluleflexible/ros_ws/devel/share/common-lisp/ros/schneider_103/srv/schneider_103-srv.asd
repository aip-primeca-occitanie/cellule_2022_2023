
(cl:in-package :asdf)

(defsystem "schneider_103-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Retour_cellule_103" :depends-on ("_package_Retour_cellule_103"))
    (:file "_package_Retour_cellule_103" :depends-on ("_package"))
  ))