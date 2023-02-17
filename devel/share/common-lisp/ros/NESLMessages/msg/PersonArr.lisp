; Auto-generated. Do not edit!


(cl:in-package NESLMessages-msg)


;//! \htmlinclude PersonArr.msg.html

(cl:defclass <PersonArr> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (personArr
    :reader personArr
    :initarg :personArr
    :type (cl:vector NESLMessages-msg:Person)
   :initform (cl:make-array 0 :element-type 'NESLMessages-msg:Person :initial-element (cl:make-instance 'NESLMessages-msg:Person))))
)

(cl:defclass PersonArr (<PersonArr>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonArr>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonArr)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name NESLMessages-msg:<PersonArr> is deprecated: use NESLMessages-msg:PersonArr instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PersonArr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:header-val is deprecated.  Use NESLMessages-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'personArr-val :lambda-list '(m))
(cl:defmethod personArr-val ((m <PersonArr>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:personArr-val is deprecated.  Use NESLMessages-msg:personArr instead.")
  (personArr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonArr>) ostream)
  "Serializes a message object of type '<PersonArr>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'personArr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'personArr))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonArr>) istream)
  "Deserializes a message object of type '<PersonArr>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'personArr) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'personArr)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'NESLMessages-msg:Person))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonArr>)))
  "Returns string type for a message object of type '<PersonArr>"
  "NESLMessages/PersonArr")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonArr)))
  "Returns string type for a message object of type 'PersonArr"
  "NESLMessages/PersonArr")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonArr>)))
  "Returns md5sum for a message object of type '<PersonArr>"
  "6fc894af3e67e11ac542d4784fd5fbb1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonArr)))
  "Returns md5sum for a message object of type 'PersonArr"
  "6fc894af3e67e11ac542d4784fd5fbb1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonArr>)))
  "Returns full string definition for message of type '<PersonArr>"
  (cl:format cl:nil "Header header~%Person[] personArr~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: NESLMessages/Person~%float64[] colorArr~%int32 personID~%NeslCoord personCoord~%bool accountedFor~%bool talking~%float32 bbx~%float32 bby~%float32 bbz~%~%================================================================================~%MSG: NESLMessages/NeslCoord~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonArr)))
  "Returns full string definition for message of type 'PersonArr"
  (cl:format cl:nil "Header header~%Person[] personArr~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: NESLMessages/Person~%float64[] colorArr~%int32 personID~%NeslCoord personCoord~%bool accountedFor~%bool talking~%float32 bbx~%float32 bby~%float32 bbz~%~%================================================================================~%MSG: NESLMessages/NeslCoord~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonArr>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'personArr) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonArr>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonArr
    (cl:cons ':header (header msg))
    (cl:cons ':personArr (personArr msg))
))
