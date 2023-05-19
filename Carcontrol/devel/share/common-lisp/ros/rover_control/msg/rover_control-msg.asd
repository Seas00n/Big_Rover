
(cl:in-package :asdf)

(defsystem "rover_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "corner" :depends-on ("_package_corner"))
    (:file "_package_corner" :depends-on ("_package"))
  ))