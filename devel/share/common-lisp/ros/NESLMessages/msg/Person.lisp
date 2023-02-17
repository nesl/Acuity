; Auto-generated. Do not edit!


(cl:in-package NESLMessages-msg)


;//! \htmlinclude Person.msg.html

(cl:defclass <Person> (roslisp-msg-protocol:ros-message)
  ((colorArr
    :reader colorArr
    :initarg :colorArr
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (personID
    :reader personID
    :initarg :personID
    :type cl:integer
    :initform 0)
   (personCoord
    :reader personCoord
    :initarg :personCoord
    :type NESLMessages-msg:NeslCoord
    :initform (cl:make-instance 'NESLMessages-msg:NeslCoord))
   (accountedFor
    :reader accountedFor
    :initarg :accountedFor
    :type cl:boolean
    :initform cl:nil)
   (talking
    :reader talking
    :initarg :talking
    :type cl:boolean
    :initform cl:nil)
   (bbx
    :reader bbx
    :initarg :bbx
    :type cl:float
    :initform 0.0)
   (bby
    :reader bby
    :initarg :bby
    :type cl:float
    :initform 0.0)
   (bbz
    :reader bbz
    :initarg :bbz
    :type cl:float
    :initform 0.0))
)

(cl:defclass Person (<Person>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Person>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Person)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name NESLMessages-msg:<Person> is deprecated: use NESLMessages-msg:Person instead.")))

(cl:ensure-generic-function 'colorArr-val :lambda-list '(m))
(cl:defmethod colorArr-val ((m <Person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:colorArr-val is deprecated.  Use NESLMessages-msg:colorArr instead.")
  (colorArr m))

(cl:ensure-generic-function 'personID-val :lambda-list '(m))
(cl:defmethod personID-val ((m <Person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:personID-val is deprecated.  Use NESLMessages-msg:personID instead.")
  (personID m))

(cl:ensure-generic-function 'personCoord-val :lambda-list '(m))
(cl:defmethod personCoord-val ((m <Person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:personCoord-val is deprecated.  Use NESLMessages-msg:personCoord instead.")
  (personCoord m))

(cl:ensure-generic-function 'accountedFor-val :lambda-list '(m))
(cl:defmethod accountedFor-val ((m <Person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:accountedFor-val is deprecated.  Use NESLMessages-msg:accountedFor instead.")
  (accountedFor m))

(cl:ensure-generic-function 'talking-val :lambda-list '(m))
(cl:defmethod talking-val ((m <Person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:talking-val is deprecated.  Use NESLMessages-msg:talking instead.")
  (talking m))

(cl:ensure-generic-function 'bbx-val :lambda-list '(m))
(cl:defmethod bbx-val ((m <Person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:bbx-val is deprecated.  Use NESLMessages-msg:bbx instead.")
  (bbx m))

(cl:ensure-generic-function 'bby-val :lambda-list '(m))
(cl:defmethod bby-val ((m <Person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:bby-val is deprecated.  Use NESLMessages-msg:bby instead.")
  (bby m))

(cl:ensure-generic-function 'bbz-val :lambda-list '(m))
(cl:defmethod bbz-val ((m <Person>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader NESLMessages-msg:bbz-val is deprecated.  Use NESLMessages-msg:bbz instead.")
  (bbz m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Person>) ostream)
  "Serializes a message object of type '<Person>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'colorArr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'colorArr))
  (cl:let* ((signed (cl:slot-value msg 'personID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'personCoord) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'accountedFor) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'talking) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bby))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bbz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Person>) istream)
  "Deserializes a message object of type '<Person>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'colorArr) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'colorArr)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'personID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'personCoord) istream)
    (cl:setf (cl:slot-value msg 'accountedFor) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'talking) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bby) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bbz) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Person>)))
  "Returns string type for a message object of type '<Person>"
  "NESLMessages/Person")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Person)))
  "Returns string type for a message object of type 'Person"
  "NESLMessages/Person")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Person>)))
  "Returns md5sum for a message object of type '<Person>"
  "07061084615db4a1b1a7a2a5b8b45019")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Person)))
  "Returns md5sum for a message object of type 'Person"
  "07061084615db4a1b1a7a2a5b8b45019")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Person>)))
  "Returns full string definition for message of type '<Person>"
  (cl:format cl:nil "float64[] colorArr~%int32 personID~%NeslCoord personCoord~%bool accountedFor~%bool talking~%float32 bbx~%float32 bby~%float32 bbz~%~%================================================================================~%MSG: NESLMessages/NeslCoord~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Person)))
  "Returns full string definition for message of type 'Person"
  (cl:format cl:nil "float64[] colorArr~%int32 personID~%NeslCoord personCoord~%bool accountedFor~%bool talking~%float32 bbx~%float32 bby~%float32 bbz~%~%================================================================================~%MSG: NESLMessages/NeslCoord~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Person>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'colorArr) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'personCoord))
     1
     1
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Person>))
  "Converts a ROS message object to a list"
  (cl:list 'Person
    (cl:cons ':colorArr (colorArr msg))
    (cl:cons ':personID (personID msg))
    (cl:cons ':personCoord (personCoord msg))
    (cl:cons ':accountedFor (accountedFor msg))
    (cl:cons ':talking (talking msg))
    (cl:cons ':bbx (bbx msg))
    (cl:cons ':bby (bby msg))
    (cl:cons ':bbz (bbz msg))
))
