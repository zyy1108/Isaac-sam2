
(cl:in-package :asdf)

(defsystem "img_seg-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Segment" :depends-on ("_package_Segment"))
    (:file "_package_Segment" :depends-on ("_package"))
  ))