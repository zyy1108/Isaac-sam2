;; Auto-generated. Do not edit!


(when (boundp 'img_seg::Segment)
  (if (not (find-package "IMG_SEG"))
    (make-package "IMG_SEG"))
  (shadow 'Segment (find-package "IMG_SEG")))
(unless (find-package "IMG_SEG::SEGMENT")
  (make-package "IMG_SEG::SEGMENT"))
(unless (find-package "IMG_SEG::SEGMENTREQUEST")
  (make-package "IMG_SEG::SEGMENTREQUEST"))
(unless (find-package "IMG_SEG::SEGMENTRESPONSE")
  (make-package "IMG_SEG::SEGMENTRESPONSE"))

(in-package "ROS")





(defclass img_seg::SegmentRequest
  :super ros::object
  :slots ())

(defmethod img_seg::SegmentRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass img_seg::SegmentResponse
  :super ros::object
  :slots ())

(defmethod img_seg::SegmentResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass img_seg::Segment
  :super ros::object
  :slots ())

(setf (get img_seg::Segment :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get img_seg::Segment :datatype-) "img_seg/Segment")
(setf (get img_seg::Segment :request) img_seg::SegmentRequest)
(setf (get img_seg::Segment :response) img_seg::SegmentResponse)

(defmethod img_seg::SegmentRequest
  (:response () (instance img_seg::SegmentResponse :init)))

(setf (get img_seg::SegmentRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get img_seg::SegmentRequest :datatype-) "img_seg/SegmentRequest")
(setf (get img_seg::SegmentRequest :definition-)
      "
---
")

(setf (get img_seg::SegmentResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get img_seg::SegmentResponse :datatype-) "img_seg/SegmentResponse")
(setf (get img_seg::SegmentResponse :definition-)
      "
---
")



(provide :img_seg/Segment "d41d8cd98f00b204e9800998ecf8427e")


