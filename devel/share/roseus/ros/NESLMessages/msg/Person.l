;; Auto-generated. Do not edit!


(when (boundp 'NESLMessages::Person)
  (if (not (find-package "NESLMESSAGES"))
    (make-package "NESLMESSAGES"))
  (shadow 'Person (find-package "NESLMESSAGES")))
(unless (find-package "NESLMESSAGES::PERSON")
  (make-package "NESLMESSAGES::PERSON"))

(in-package "ROS")
;;//! \htmlinclude Person.msg.html


(defclass NESLMessages::Person
  :super ros::object
  :slots (_colorArr _personID _personCoord _accountedFor _talking _bbx _bby _bbz ))

(defmethod NESLMessages::Person
  (:init
   (&key
    ((:colorArr __colorArr) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:personID __personID) 0)
    ((:personCoord __personCoord) (instance NESLMessages::NeslCoord :init))
    ((:accountedFor __accountedFor) nil)
    ((:talking __talking) nil)
    ((:bbx __bbx) 0.0)
    ((:bby __bby) 0.0)
    ((:bbz __bbz) 0.0)
    )
   (send-super :init)
   (setq _colorArr __colorArr)
   (setq _personID (round __personID))
   (setq _personCoord __personCoord)
   (setq _accountedFor __accountedFor)
   (setq _talking __talking)
   (setq _bbx (float __bbx))
   (setq _bby (float __bby))
   (setq _bbz (float __bbz))
   self)
  (:colorArr
   (&optional __colorArr)
   (if __colorArr (setq _colorArr __colorArr)) _colorArr)
  (:personID
   (&optional __personID)
   (if __personID (setq _personID __personID)) _personID)
  (:personCoord
   (&rest __personCoord)
   (if (keywordp (car __personCoord))
       (send* _personCoord __personCoord)
     (progn
       (if __personCoord (setq _personCoord (car __personCoord)))
       _personCoord)))
  (:accountedFor
   (&optional (__accountedFor :null))
   (if (not (eq __accountedFor :null)) (setq _accountedFor __accountedFor)) _accountedFor)
  (:talking
   (&optional (__talking :null))
   (if (not (eq __talking :null)) (setq _talking __talking)) _talking)
  (:bbx
   (&optional __bbx)
   (if __bbx (setq _bbx __bbx)) _bbx)
  (:bby
   (&optional __bby)
   (if __bby (setq _bby __bby)) _bby)
  (:bbz
   (&optional __bbz)
   (if __bbz (setq _bbz __bbz)) _bbz)
  (:serialization-length
   ()
   (+
    ;; float64[] _colorArr
    (* 8    (length _colorArr)) 4
    ;; int32 _personID
    4
    ;; NESLMessages/NeslCoord _personCoord
    (send _personCoord :serialization-length)
    ;; bool _accountedFor
    1
    ;; bool _talking
    1
    ;; float32 _bbx
    4
    ;; float32 _bby
    4
    ;; float32 _bbz
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _colorArr
     (write-long (length _colorArr) s)
     (dotimes (i (length _colorArr))
       (sys::poke (elt _colorArr i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; int32 _personID
       (write-long _personID s)
     ;; NESLMessages/NeslCoord _personCoord
       (send _personCoord :serialize s)
     ;; bool _accountedFor
       (if _accountedFor (write-byte -1 s) (write-byte 0 s))
     ;; bool _talking
       (if _talking (write-byte -1 s) (write-byte 0 s))
     ;; float32 _bbx
       (sys::poke _bbx (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bby
       (sys::poke _bby (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bbz
       (sys::poke _bbz (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _colorArr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _colorArr (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _colorArr i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; int32 _personID
     (setq _personID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; NESLMessages/NeslCoord _personCoord
     (send _personCoord :deserialize buf ptr-) (incf ptr- (send _personCoord :serialization-length))
   ;; bool _accountedFor
     (setq _accountedFor (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _talking
     (setq _talking (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _bbx
     (setq _bbx (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bby
     (setq _bby (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bbz
     (setq _bbz (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get NESLMessages::Person :md5sum-) "07061084615db4a1b1a7a2a5b8b45019")
(setf (get NESLMessages::Person :datatype-) "NESLMessages/Person")
(setf (get NESLMessages::Person :definition-)
      "float64[] colorArr
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



(provide :NESLMessages/Person "07061084615db4a1b1a7a2a5b8b45019")


