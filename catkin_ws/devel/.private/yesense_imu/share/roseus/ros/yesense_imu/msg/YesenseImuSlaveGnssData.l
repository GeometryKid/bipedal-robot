;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuSlaveGnssData)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuSlaveGnssData (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUSLAVEGNSSDATA")
  (make-package "YESENSE_IMU::YESENSEIMUSLAVEGNSSDATA"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuSlaveGnssData.msg.html


(defclass yesense_imu::YesenseImuSlaveGnssData
  :super ros::object
  :slots (_dual_ant_yaw _dual_ant_yaw_error _dual_ant_baseline_len ))

(defmethod yesense_imu::YesenseImuSlaveGnssData
  (:init
   (&key
    ((:dual_ant_yaw __dual_ant_yaw) 0.0)
    ((:dual_ant_yaw_error __dual_ant_yaw_error) 0.0)
    ((:dual_ant_baseline_len __dual_ant_baseline_len) 0.0)
    )
   (send-super :init)
   (setq _dual_ant_yaw (float __dual_ant_yaw))
   (setq _dual_ant_yaw_error (float __dual_ant_yaw_error))
   (setq _dual_ant_baseline_len (float __dual_ant_baseline_len))
   self)
  (:dual_ant_yaw
   (&optional __dual_ant_yaw)
   (if __dual_ant_yaw (setq _dual_ant_yaw __dual_ant_yaw)) _dual_ant_yaw)
  (:dual_ant_yaw_error
   (&optional __dual_ant_yaw_error)
   (if __dual_ant_yaw_error (setq _dual_ant_yaw_error __dual_ant_yaw_error)) _dual_ant_yaw_error)
  (:dual_ant_baseline_len
   (&optional __dual_ant_baseline_len)
   (if __dual_ant_baseline_len (setq _dual_ant_baseline_len __dual_ant_baseline_len)) _dual_ant_baseline_len)
  (:serialization-length
   ()
   (+
    ;; float32 _dual_ant_yaw
    4
    ;; float32 _dual_ant_yaw_error
    4
    ;; float32 _dual_ant_baseline_len
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _dual_ant_yaw
       (sys::poke _dual_ant_yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dual_ant_yaw_error
       (sys::poke _dual_ant_yaw_error (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dual_ant_baseline_len
       (sys::poke _dual_ant_baseline_len (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _dual_ant_yaw
     (setq _dual_ant_yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dual_ant_yaw_error
     (setq _dual_ant_yaw_error (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dual_ant_baseline_len
     (setq _dual_ant_baseline_len (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuSlaveGnssData :md5sum-) "8343f347c57f056bf546331417a9a305")
(setf (get yesense_imu::YesenseImuSlaveGnssData :datatype-) "yesense_imu/YesenseImuSlaveGnssData")
(setf (get yesense_imu::YesenseImuSlaveGnssData :definition-)
      "float32 dual_ant_yaw
float32 dual_ant_yaw_error
float32 dual_ant_baseline_len
")



(provide :yesense_imu/YesenseImuSlaveGnssData "8343f347c57f056bf546331417a9a305")


