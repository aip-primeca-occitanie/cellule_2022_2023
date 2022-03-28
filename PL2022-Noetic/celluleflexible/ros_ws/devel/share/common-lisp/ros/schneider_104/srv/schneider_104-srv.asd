
(cl:in-package :asdf)

(defsystem "schneider_104-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Retour_cellule_104" :depends-on ("_package_Retour_cellule_104"))
    (:file "_package_Retour_cellule_104" :depends-on ("_package"))
  ))