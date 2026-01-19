;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuCmdResp)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuCmdResp (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUCMDRESP")
  (make-package "YESENSE_IMU::YESENSEIMUCMDRESP"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuCmdResp.msg.html


(defclass yesense_imu::YesenseImuCmdResp
  :super ros::object
  :slots (_id _cmd _success _msg _data ))

(defmethod yesense_imu::YesenseImuCmdResp
  (:init
   (&key
    ((:id __id) "")
    ((:cmd __cmd) "")
    ((:success __success) nil)
    ((:msg __msg) "")
    ((:data __data) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _id (string __id))
   (setq _cmd (string __cmd))
   (setq _success __success)
   (setq _msg (string __msg))
   (setq _data __data)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _id
    4 (length _id)
    ;; string _cmd
    4 (length _cmd)
    ;; bool _success
    1
    ;; string _msg
    4 (length _msg)
    ;; uint8[] _data
    (* 1    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _id
       (write-long (length _id) s) (princ _id s)
     ;; string _cmd
       (write-long (length _cmd) s) (princ _cmd s)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _msg
       (write-long (length _msg) s) (princ _msg s)
     ;; uint8[] _data
     (write-long (length _data) s)
     (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _cmd
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _msg
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _msg (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8[] _data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _data (make-array n :element-type :char))
     (replace _data buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuCmdResp :md5sum-) "340d2819898517e7a076960b9322a4b4")
(setf (get yesense_imu::YesenseImuCmdResp :datatype-) "yesense_imu/YesenseImuCmdResp")
(setf (get yesense_imu::YesenseImuCmdResp :definition-)
      "string id
string cmd
bool success
string msg
uint8[] data
")



(provide :yesense_imu/YesenseImuCmdResp "340d2819898517e7a076960b9322a4b4")


