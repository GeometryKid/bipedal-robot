;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuGnssData)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuGnssData (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUGNSSDATA")
  (make-package "YESENSE_IMU::YESENSEIMUGNSSDATA"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuGnssData.msg.html


(defclass yesense_imu::YesenseImuGnssData
  :super ros::object
  :slots (_master _slave ))

(defmethod yesense_imu::YesenseImuGnssData
  (:init
   (&key
    ((:master __master) (instance yesense_imu::YesenseImuMasterGnssData :init))
    ((:slave __slave) (instance yesense_imu::YesenseImuSlaveGnssData :init))
    )
   (send-super :init)
   (setq _master __master)
   (setq _slave __slave)
   self)
  (:master
   (&rest __master)
   (if (keywordp (car __master))
       (send* _master __master)
     (progn
       (if __master (setq _master (car __master)))
       _master)))
  (:slave
   (&rest __slave)
   (if (keywordp (car __slave))
       (send* _slave __slave)
     (progn
       (if __slave (setq _slave (car __slave)))
       _slave)))
  (:serialization-length
   ()
   (+
    ;; yesense_imu/YesenseImuMasterGnssData _master
    (send _master :serialization-length)
    ;; yesense_imu/YesenseImuSlaveGnssData _slave
    (send _slave :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; yesense_imu/YesenseImuMasterGnssData _master
       (send _master :serialize s)
     ;; yesense_imu/YesenseImuSlaveGnssData _slave
       (send _slave :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; yesense_imu/YesenseImuMasterGnssData _master
     (send _master :deserialize buf ptr-) (incf ptr- (send _master :serialization-length))
   ;; yesense_imu/YesenseImuSlaveGnssData _slave
     (send _slave :deserialize buf ptr-) (incf ptr- (send _slave :serialization-length))
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuGnssData :md5sum-) "4ae993c6fa242f94866ceb31ffc6e7d6")
(setf (get yesense_imu::YesenseImuGnssData :datatype-) "yesense_imu/YesenseImuGnssData")
(setf (get yesense_imu::YesenseImuGnssData :definition-)
      "yesense_imu/YesenseImuMasterGnssData master
yesense_imu/YesenseImuSlaveGnssData slave
================================================================================
MSG: yesense_imu/YesenseImuMasterGnssData
yesense_imu/YesenseImuUtcTime utc_time
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
================================================================================
MSG: yesense_imu/YesenseImuSlaveGnssData
float32 dual_ant_yaw
float32 dual_ant_yaw_error
float32 dual_ant_baseline_len
")



(provide :yesense_imu/YesenseImuGnssData "4ae993c6fa242f94866ceb31ffc6e7d6")


