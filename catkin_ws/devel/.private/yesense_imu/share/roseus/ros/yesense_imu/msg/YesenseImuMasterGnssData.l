;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuMasterGnssData)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuMasterGnssData (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUMASTERGNSSDATA")
  (make-package "YESENSE_IMU::YESENSEIMUMASTERGNSSDATA"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuMasterGnssData.msg.html


(defclass yesense_imu::YesenseImuMasterGnssData
  :super ros::object
  :slots (_utc_time _location _location_error _speed _yaw _status _star_cnt _p_dop _site_id ))

(defmethod yesense_imu::YesenseImuMasterGnssData
  (:init
   (&key
    ((:utc_time __utc_time) (instance yesense_imu::YesenseImuUtcTime :init))
    ((:location __location) (instance yesense_imu::YesenseImuLocation :init))
    ((:location_error __location_error) (instance yesense_imu::YesenseImuLocation :init))
    ((:speed __speed) 0.0)
    ((:yaw __yaw) 0.0)
    ((:status __status) 0)
    ((:star_cnt __star_cnt) 0)
    ((:p_dop __p_dop) 0.0)
    ((:site_id __site_id) 0)
    )
   (send-super :init)
   (setq _utc_time __utc_time)
   (setq _location __location)
   (setq _location_error __location_error)
   (setq _speed (float __speed))
   (setq _yaw (float __yaw))
   (setq _status (round __status))
   (setq _star_cnt (round __star_cnt))
   (setq _p_dop (float __p_dop))
   (setq _site_id (round __site_id))
   self)
  (:utc_time
   (&rest __utc_time)
   (if (keywordp (car __utc_time))
       (send* _utc_time __utc_time)
     (progn
       (if __utc_time (setq _utc_time (car __utc_time)))
       _utc_time)))
  (:location
   (&rest __location)
   (if (keywordp (car __location))
       (send* _location __location)
     (progn
       (if __location (setq _location (car __location)))
       _location)))
  (:location_error
   (&rest __location_error)
   (if (keywordp (car __location_error))
       (send* _location_error __location_error)
     (progn
       (if __location_error (setq _location_error (car __location_error)))
       _location_error)))
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:star_cnt
   (&optional __star_cnt)
   (if __star_cnt (setq _star_cnt __star_cnt)) _star_cnt)
  (:p_dop
   (&optional __p_dop)
   (if __p_dop (setq _p_dop __p_dop)) _p_dop)
  (:site_id
   (&optional __site_id)
   (if __site_id (setq _site_id __site_id)) _site_id)
  (:serialization-length
   ()
   (+
    ;; yesense_imu/YesenseImuUtcTime _utc_time
    (send _utc_time :serialization-length)
    ;; yesense_imu/YesenseImuLocation _location
    (send _location :serialization-length)
    ;; yesense_imu/YesenseImuLocation _location_error
    (send _location_error :serialization-length)
    ;; float32 _speed
    4
    ;; float32 _yaw
    4
    ;; uint8 _status
    1
    ;; uint8 _star_cnt
    1
    ;; float32 _p_dop
    4
    ;; uint8 _site_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; yesense_imu/YesenseImuUtcTime _utc_time
       (send _utc_time :serialize s)
     ;; yesense_imu/YesenseImuLocation _location
       (send _location :serialize s)
     ;; yesense_imu/YesenseImuLocation _location_error
       (send _location_error :serialize s)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _status
       (write-byte _status s)
     ;; uint8 _star_cnt
       (write-byte _star_cnt s)
     ;; float32 _p_dop
       (sys::poke _p_dop (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _site_id
       (write-byte _site_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; yesense_imu/YesenseImuUtcTime _utc_time
     (send _utc_time :deserialize buf ptr-) (incf ptr- (send _utc_time :serialization-length))
   ;; yesense_imu/YesenseImuLocation _location
     (send _location :deserialize buf ptr-) (incf ptr- (send _location :serialization-length))
   ;; yesense_imu/YesenseImuLocation _location_error
     (send _location_error :deserialize buf ptr-) (incf ptr- (send _location_error :serialization-length))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _status
     (setq _status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _star_cnt
     (setq _star_cnt (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _p_dop
     (setq _p_dop (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _site_id
     (setq _site_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuMasterGnssData :md5sum-) "707ba72b8fe698890901dc5ceaf55863")
(setf (get yesense_imu::YesenseImuMasterGnssData :datatype-) "yesense_imu/YesenseImuMasterGnssData")
(setf (get yesense_imu::YesenseImuMasterGnssData :definition-)
      "yesense_imu/YesenseImuUtcTime utc_time
yesense_imu/YesenseImuLocation location
yesense_imu/YesenseImuLocation location_error
float32 speed
float32 yaw
uint8 status
uint8 star_cnt
float32 p_dop
uint8 site_id
================================================================================
MSG: yesense_imu/YesenseImuUtcTime
uint16 year
uint8 month
uint8 date
uint8 hour
uint8 min
uint8 sec
uint16 ms
================================================================================
MSG: yesense_imu/YesenseImuLocation
float64 longtidue
float64 latitude
float32 altidue
")



(provide :yesense_imu/YesenseImuMasterGnssData "707ba72b8fe698890901dc5ceaf55863")


