;; Auto-generated. Do not edit!


(when (boundp 'NESLMessages::PersonArr)
  (if (not (find-package "NESLMESSAGES"))
    (make-package "NESLMESSAGES"))
  (shadow 'PersonArr (find-package "NESLMESSAGES")))
(unless (find-package "NESLMESSAGES::PERSONARR")
  (make-package "NESLMESSAGES::PERSONARR"))

(in-package "ROS")
;;//! \htmlinclude PersonArr.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass NESLMessages::PersonArr
  :super ros::object
  :slots (_header _personArr ))

(defmethod NESLMessages::PersonArr
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:personArr __personArr) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _personArr __personArr)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:personArr
   (&rest __personArr)
   (if (keywordp (car __personArr))
       (send* _personArr __personArr)
     (progn
       (if __personArr (setq _personArr (car __personArr)))
       _personArr)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; NESLMessages/Person[] _personArr
    (apply #'+ (send-all _personArr :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; NESLMessages/Person[] _personArr
     (write-long (length _personArr) s)
     (dolist (elem _personArr)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; NESLMessages/Person[] _personArr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _personArr (let (r) (dotimes (i n) (push (instance NESLMessages::Person :init) r)) r))
     (dolist (elem- _personArr)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get NESLMessages::PersonArr :md5sum-) "6fc894af3e67e11ac542d4784fd5fbb1")
(setf (get NESLMessages::PersonArr :datatype-) "NESLMessages/PersonArr")
(setf (get NESLMessages::PersonArr :definition-)
      "Header header
Person[] personArr


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: NESLMessages/Person
float64[] colorArr
int32 personID
NeslCoord personCoord
bool accountedFor
bool talking
float32 bbx
float32 bby
float32 bbz

================================================================================
MSG: NESLMessages/NeslCoord
float64 x
float64 y
float64 z

")



(provide :NESLMessages/PersonArr "6fc894af3e67e11ac542d4784fd5fbb1")


