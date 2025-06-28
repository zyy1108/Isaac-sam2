; Auto-generated. Do not edit!


(cl:in-package img_seg-srv)


;//! \htmlinclude Segment-request.msg.html

(cl:defclass <Segment-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Segment-request (<Segment-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Segment-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Segment-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name img_seg-srv:<Segment-request> is deprecated: use img_seg-srv:Segment-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Segment-request>) ostream)
  "Serializes a message object of type '<Segment-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Segment-request>) istream)
  "Deserializes a message object of type '<Segment-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Segment-request>)))
  "Returns string type for a service object of type '<Segment-request>"
  "img_seg/SegmentRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Segment-request)))
  "Returns string type for a service object of type 'Segment-request"
  "img_seg/SegmentRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Segment-request>)))
  "Returns md5sum for a message object of type '<Segment-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Segment-request)))
  "Returns md5sum for a message object of type 'Segment-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Segment-request>)))
  "Returns full string definition for message of type '<Segment-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Segment-request)))
  "Returns full string definition for message of type 'Segment-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Segment-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Segment-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Segment-request
))
;//! \htmlinclude Segment-response.msg.html

(cl:defclass <Segment-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Segment-response (<Segment-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Segment-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Segment-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name img_seg-srv:<Segment-response> is deprecated: use img_seg-srv:Segment-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Segment-response>) ostream)
  "Serializes a message object of type '<Segment-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Segment-response>) istream)
  "Deserializes a message object of type '<Segment-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Segment-response>)))
  "Returns string type for a service object of type '<Segment-response>"
  "img_seg/SegmentResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Segment-response)))
  "Returns string type for a service object of type 'Segment-response"
  "img_seg/SegmentResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Segment-response>)))
  "Returns md5sum for a message object of type '<Segment-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Segment-response)))
  "Returns md5sum for a message object of type 'Segment-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Segment-response>)))
  "Returns full string definition for message of type '<Segment-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Segment-response)))
  "Returns full string definition for message of type 'Segment-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Segment-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Segment-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Segment-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Segment)))
  'Segment-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Segment)))
  'Segment-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Segment)))
  "Returns string type for a service object of type '<Segment>"
  "img_seg/Segment")