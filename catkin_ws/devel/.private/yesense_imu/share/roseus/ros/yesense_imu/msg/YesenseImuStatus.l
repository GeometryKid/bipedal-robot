;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuStatus)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuStatus (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUSTATUS")
  (make-package "YESENSE_IMU::YESENSEIMUSTATUS"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuStatus.msg.html


(defclass yesense_imu::YesenseImuStatus
  :super ros::object
  :slots (_fusion_status _gnss_status ))

(defmethod yesense_imu::YesenseImuStatus
  (:init
   (&key
    ((:fusion_status __fusion_status) 0)
    ((:gnss_status __gnss_status) 0)
    )
   (send-super :init)
   (setq _fusion_status (round __fusion_status))
   (setq _gnss_status (round __gnss_status))
   self)
  (:fusion_status
   (&optional __fusion_status)
   (if __fusion_status (setq _fusion_status __fusion_status)) _fusion_status)
  (:gnss_status
   (&optional __gnss_status)
   (if __gnss_status (setq _gnss_status __gnss_status)) _gnss_status)
  (:serialization-length
   ()
   (+
    ;; uint8 _fusion_status
    1
    ;; uint8 _gnss_status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _fusion_status
       (write-byte _fusion_status s)
     ;; uint8 _gnss_status
       (write-byte _gnss_status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _fusion_status
     (setq _fusion_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _gnss_status
     (setq _gnss_status (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuStatus :md5sum-) "ce0abe5eeaff955681f09594f0484f51")
(setf (get yesense_imu::YesenseImuStatus :datatype-) "yesense_imu/YesenseImuStatus")
(setf (get yesense_imu::YesenseImuStatus :definition-)
      "uint8 fusion_status
uint8 gnss_status
")



(provide :yesense_imu/YesenseImuStatus "ce0abe5eeaff955681f09594f0484f51")


