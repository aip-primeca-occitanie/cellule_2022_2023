
(cl:in-package :asdf)

(defsystem "commande_locale-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SrvAddProduct" :depends-on ("_package_SrvAddProduct"))
    (:file "_package_SrvAddProduct" :depends-on ("_package"))
    (:file "SrvAddProductPushBack" :depends-on ("_package_SrvAddProductPushBack"))
    (:file "_package_SrvAddProductPushBack" :depends-on ("_package"))
    (:file "SrvFinInit" :depends-on ("_package_SrvFinInit"))
    (:file "_package_SrvFinInit" :depends-on ("_package"))
  ))