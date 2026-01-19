;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuLocation)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuLocation (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMULOCATION")
  (make-package "YESENSE_IMU::YESENSEIMULOCATION"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuLocation.msg.html


(defclass yesense_imu::YesenseImuLocation
  :super ros::object
  :slots (_longtidue _latitude _altidue ))

(defmethod yesense_imu::YesenseImuLocation
  (:init
   (&key
    ((:longtidue __longtidue) 0.0)
    ((:latitude __latitude) 0.0)
    ((:altidue __altidue) 0.0)
    )
   (send-super :init)
   (setq _longtidue (float __longtidue))
   (setq _latitude (float __latitude))
   (setq _altidue (float __altidue))
   self)
  (:longtidue
   (&optional __longtidue)
   (if __longtidue (setq _longtidue __longtidue)) _longtidue)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:altidue
   (&optional __altidue)
   (if __altidue (setq _altidue __altidue)) _altidue)
  (:serialization-length
   ()
   (+
    ;; float64 _longtidue
    8
    ;; float64 _latitude
    8
    ;; float32 _altidue
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _longtidue
       (sys::poke _longtidue (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float32 _altidue
       (sys::poke _altidue (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _longtidue
     (setq _longtidue (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float32 _altidue
     (setq _altidue (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuLocation :md5sum-) "5cf93867aa740e2448c799a9da22809d")
(setf (get yesense_imu::YesenseImuLocation :datatype-) "yesense_imu/YesenseImuLocation")
(setf (get yesense_imu::YesenseImuLocation :definition-)
      "float64 longtidue
float64 latitude
float32 altidue
")



(provide :yesense_imu/YesenseImuLocation "5cf93867aa740e2448c799a9da22809d")


