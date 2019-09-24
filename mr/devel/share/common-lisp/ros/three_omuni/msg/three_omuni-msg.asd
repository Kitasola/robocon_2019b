
(cl:in-package :asdf)

(defsystem "three_omuni-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "button" :depends-on ("_package_button"))
    (:file "_package_button" :depends-on ("_package"))
  ))