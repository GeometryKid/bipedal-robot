;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuUtcTime)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuUtcTime (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUUTCTIME")
  (make-package "YESENSE_IMU::YESENSEIMUUTCTIME"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuUtcTime.msg.html


(defclass yesense_imu::YesenseImuUtcTime
  :super ros::object
  :slots (_year _month _date _hour _min _sec _ms ))

(defmethod yesense_imu::YesenseImuUtcTime
  (:init
   (&key
    ((:year __year) 0)
    ((:month __month) 0)
    ((:date __date) 0)
    ((:hour __hour) 0)
    ((:min __min) 0)
    ((:sec __sec) 0)
    ((:ms __ms) 0)
    )
   (send-super :init)
   (setq _year (round __year))
   (setq _month (round __month))
   (setq _date (round __date))
   (setq _hour (round __hour))
   (setq _min (round __min))
   (setq _sec (round __sec))
   (setq _ms (round __ms))
   self)
  (:year
   (&optional __year)
   (if __year (setq _year __year)) _year)
  (:month
   (&optional __month)
   (if __month (setq _month __month)) _month)
  (:date
   (&optional __date)
   (if __date (setq _date __date)) _date)
  (:hour
   (&optional __hour)
   (if __hour (setq _hour __hour)) _hour)
  (:min
   (&optional __min)
   (if __min (setq _min __min)) _min)
  (:sec
   (&optional __sec)
   (if __sec (setq _sec __sec)) _sec)
  (:ms
   (&optional __ms)
   (if __ms (setq _ms __ms)) _ms)
  (:serialization-length
   ()
   (+
    ;; uint16 _year
    2
    ;; uint8 _month
    1
    ;; uint8 _date
    1
    ;; uint8 _hour
    1
    ;; uint8 _min
    1
    ;; uint8 _sec
    1
    ;; uint16 _ms
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _year
       (write-word _year s)
     ;; uint8 _month
       (write-byte _month s)
     ;; uint8 _date
       (write-byte _date s)
     ;; uint8 _hour
       (write-byte _hour s)
     ;; uint8 _min
       (write-byte _min s)
     ;; uint8 _sec
       (write-byte _sec s)
     ;; uint16 _ms
       (write-word _ms s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _year
     (setq _year (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint8 _month
     (setq _month (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _date
     (setq _date (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _hour
     (setq _hour (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _min
     (setq _min (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _sec
     (setq _sec (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _ms
     (setq _ms (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuUtcTime :md5sum-) "9ef427732f567725a6400541b0a0bb7e")
(setf (get yesense_imu::YesenseImuUtcTime :datatype-) "yesense_imu/YesenseImuUtcTime")
(setf (get yesense_imu::YesenseImuUtcTime :definition-)
      "uint16 year
uint8 month
uint8 date
uint8 hour
uint8 min
uint8 sec
uint16 ms
")



(provide :yesense_imu/YesenseImuUtcTime "9ef427732f567725a6400541b0a0bb7e")


