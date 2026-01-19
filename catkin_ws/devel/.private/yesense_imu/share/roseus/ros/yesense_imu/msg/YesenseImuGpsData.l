;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuGpsData)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuGpsData (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUGPSDATA")
  (make-package "YESENSE_IMU::YESENSEIMUGPSDATA"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuGpsData.msg.html


(defclass yesense_imu::YesenseImuGpsData
  :super ros::object
  :slots (_raw_data ))

(defmethod yesense_imu::YesenseImuGpsData
  (:init
   (&key
    ((:raw_data __raw_data) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _raw_data __raw_data)
   self)
  (:raw_data
   (&optional __raw_data)
   (if __raw_data (setq _raw_data __raw_data)) _raw_data)
  (:serialization-length
   ()
   (+
    ;; string[] _raw_data
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _raw_data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _raw_data
     (write-long (length _raw_data) s)
     (dolist (elem _raw_data)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _raw_data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _raw_data (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _raw_data i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuGpsData :md5sum-) "a93dac2a6e802ac9f45a79f2b894d63d")
(setf (get yesense_imu::YesenseImuGpsData :datatype-) "yesense_imu/YesenseImuGpsData")
(setf (get yesense_imu::YesenseImuGpsData :definition-)
      "string[] raw_data
")



(provide :yesense_imu/YesenseImuGpsData "a93dac2a6e802ac9f45a79f2b894d63d")


