
(cl:in-package :asdf)

(defsystem "legged_wbc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "nvector3f" :depends-on ("_package_nvector3f"))
    (:file "_package_nvector3f" :depends-on ("_package"))
    (:file "vector3f" :depends-on ("_package_vector3f"))
    (:file "_package_vector3f" :depends-on ("_package"))
  ))