
(cl:in-package :asdf)

(defsystem "NESLMessages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "NeslCoord" :depends-on ("_package_NeslCoord"))
    (:file "_package_NeslCoord" :depends-on ("_package"))
    (:file "Person" :depends-on ("_package_Person"))
    (:file "_package_Person" :depends-on ("_package"))
    (:file "PersonArr" :depends-on ("_package_PersonArr"))
    (:file "_package_PersonArr" :depends-on ("_package"))
  ))