;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseIMUSetting)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseIMUSetting (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUSETTING")
  (make-package "YESENSE_IMU::YESENSEIMUSETTING"))

(in-package "ROS")
;;//! \htmlinclude YesenseIMUSetting.msg.html


(defclass yesense_imu::YesenseIMUSetting
  :super ros::object
  :slots (_class _id _length _msg ))

(defmethod yesense_imu::YesenseIMUSetting
  (:init
   (&key
    ((:class __class) 0)
    ((:id __id) 0)
    ((:length __length) 0)
    ((:msg __msg) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _class (round __class))
   (setq _id (round __id))
   (setq _length (round __length))
   (setq _msg __msg)
   self)
  (:class
   (&optional __class)
   (if __class (setq _class __class)) _class)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:msg
   (&optional __msg)
   (if __msg (setq _msg __msg)) _msg)
  (:serialization-length
   ()
   (+
    ;; uint8 _class
    1
    ;; uint8 _id
    1
    ;; uint8 _length
    1
    ;; uint8[] _msg
    (* 1    (length _msg)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _class
       (write-byte _class s)
     ;; uint8 _id
       (write-byte _id s)
     ;; uint8 _length
       (write-byte _length s)
     ;; uint8[] _msg
     (write-long (length _msg) s)
     (princ _msg s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _class
     (setq _class (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _length
     (setq _length (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8[] _msg
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _msg (make-array n :element-type :char))
     (replace _msg buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get yesense_imu::YesenseIMUSetting :md5sum-) "7e7a91ef26451a647daf3866c66ac74c")
(setf (get yesense_imu::YesenseIMUSetting :datatype-) "yesense_imu/YesenseIMUSetting")
(setf (get yesense_imu::YesenseIMUSetting :definition-)
      "#uint8 CLASS_PRODUCTION_INFO   = 0
#uint8 CLASS_BAUDRATE          = 2
#uint8 CLASS_OUTPUT_FREEQUENCY = 3
#uint8 CLASS_OUTPUT_CONTENT    = 4
#uint8 CLASS_STANDARD_PARAM    = 5
#uint8 CLASS_MODE              = 77   #0x4D
#uint8 CLASS_NMEA_OUTPUT       = 78   #0x4E

#uint8 ID_REQUEST              = 0
#uint8 ID_MEMERY               = 1
#uint8 ID_FLASH                = 2

uint8 class
uint8 id
uint8 length
uint8[] msg
")



(provide :yesense_imu/YesenseIMUSetting "7e7a91ef26451a647daf3866c66ac74c")


