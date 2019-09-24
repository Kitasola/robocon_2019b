
(cl:in-package :asdf)

(defsystem "rc2019_commander-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "button" :depends-on ("_package_button"))
    (:file "_package_button" :depends-on ("_package"))
  ))