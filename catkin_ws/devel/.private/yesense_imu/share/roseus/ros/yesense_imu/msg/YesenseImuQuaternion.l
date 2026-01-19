;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuQuaternion)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuQuaternion (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUQUATERNION")
  (make-package "YESENSE_IMU::YESENSEIMUQUATERNION"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuQuaternion.msg.html


(defclass yesense_imu::YesenseImuQuaternion
  :super ros::object
  :slots (_q0 _q1 _q2 _q3 ))

(defmethod yesense_imu::YesenseImuQuaternion
  (:init
   (&key
    ((:q0 __q0) 0.0)
    ((:q1 __q1) 0.0)
    ((:q2 __q2) 0.0)
    ((:q3 __q3) 0.0)
    )
   (send-super :init)
   (setq _q0 (float __q0))
   (setq _q1 (float __q1))
   (setq _q2 (float __q2))
   (setq _q3 (float __q3))
   self)
  (:q0
   (&optional __q0)
   (if __q0 (setq _q0 __q0)) _q0)
  (:q1
   (&optional __q1)
   (if __q1 (setq _q1 __q1)) _q1)
  (:q2
   (&optional __q2)
   (if __q2 (setq _q2 __q2)) _q2)
  (:q3
   (&optional __q3)
   (if __q3 (setq _q3 __q3)) _q3)
  (:serialization-length
   ()
   (+
    ;; float32 _q0
    4
    ;; float32 _q1
    4
    ;; float32 _q2
    4
    ;; float32 _q3
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _q0
       (sys::poke _q0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q1
       (sys::poke _q1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q2
       (sys::poke _q2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _q3
       (sys::poke _q3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _q0
     (setq _q0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q1
     (setq _q1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q2
     (setq _q2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _q3
     (setq _q3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuQuaternion :md5sum-) "63ca77594d1bad6ed3441f10f9332674")
(setf (get yesense_imu::YesenseImuQuaternion :datatype-) "yesense_imu/YesenseImuQuaternion")
(setf (get yesense_imu::YesenseImuQuaternion :definition-)
      "float32 q0
float32 q1
float32 q2
float32 q3

")



(provide :yesense_imu/YesenseImuQuaternion "63ca77594d1bad6ed3441f10f9332674")


