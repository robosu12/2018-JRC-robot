
(cl:in-package :asdf)

(defsystem "id_data_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "forceData" :depends-on ("_package_forceData"))
    (:file "_package_forceData" :depends-on ("_package"))
    (:file "ID_Data" :depends-on ("_package_ID_Data"))
    (:file "_package_ID_Data" :depends-on ("_package"))
  ))