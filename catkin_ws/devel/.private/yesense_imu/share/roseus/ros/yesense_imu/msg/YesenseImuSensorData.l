;; Auto-generated. Do not edit!


(when (boundp 'yesense_imu::YesenseImuSensorData)
  (if (not (find-package "YESENSE_IMU"))
    (make-package "YESENSE_IMU"))
  (shadow 'YesenseImuSensorData (find-package "YESENSE_IMU")))
(unless (find-package "YESENSE_IMU::YESENSEIMUSENSORDATA")
  (make-package "YESENSE_IMU::YESENSEIMUSENSORDATA"))

(in-package "ROS")
;;//! \htmlinclude YesenseImuSensorData.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass yesense_imu::YesenseImuSensorData
  :super ros::object
  :slots (_temperature _sample_timestamp _sync_timestamp _accel _quaternion _eulerAngle _location ))

(defmethod yesense_imu::YesenseImuSensorData
  (:init
   (&key
    ((:temperature __temperature) 0.0)
    ((:sample_timestamp __sample_timestamp) 0)
    ((:sync_timestamp __sync_timestamp) 0)
    ((:accel __accel) (instance geometry_msgs::Accel :init))
    ((:quaternion __quaternion) (instance yesense_imu::YesenseImuQuaternion :init))
    ((:eulerAngle __eulerAngle) (instance yesense_imu::YesenseImuEulerAngle :init))
    ((:location __location) (instance yesense_imu::YesenseImuLocation :init))
    )
   (send-super :init)
   (setq _temperature (float __temperature))
   (setq _sample_timestamp (round __sample_timestamp))
   (setq _sync_timestamp (round __sync_timestamp))
   (setq _accel __accel)
   (setq _quaternion __quaternion)
   (setq _eulerAngle __eulerAngle)
   (setq _location __location)
   self)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:sample_timestamp
   (&optional __sample_timestamp)
   (if __sample_timestamp (setq _sample_timestamp __sample_timestamp)) _sample_timestamp)
  (:sync_timestamp
   (&optional __sync_timestamp)
   (if __sync_timestamp (setq _sync_timestamp __sync_timestamp)) _sync_timestamp)
  (:accel
   (&rest __accel)
   (if (keywordp (car __accel))
       (send* _accel __accel)
     (progn
       (if __accel (setq _accel (car __accel)))
       _accel)))
  (:quaternion
   (&rest __quaternion)
   (if (keywordp (car __quaternion))
       (send* _quaternion __quaternion)
     (progn
       (if __quaternion (setq _quaternion (car __quaternion)))
       _quaternion)))
  (:eulerAngle
   (&rest __eulerAngle)
   (if (keywordp (car __eulerAngle))
       (send* _eulerAngle __eulerAngle)
     (progn
       (if __eulerAngle (setq _eulerAngle (car __eulerAngle)))
       _eulerAngle)))
  (:location
   (&rest __location)
   (if (keywordp (car __location))
       (send* _location __location)
     (progn
       (if __location (setq _location (car __location)))
       _location)))
  (:serialization-length
   ()
   (+
    ;; float32 _temperature
    4
    ;; uint32 _sample_timestamp
    4
    ;; uint32 _sync_timestamp
    4
    ;; geometry_msgs/Accel _accel
    (send _accel :serialization-length)
    ;; yesense_imu/YesenseImuQuaternion _quaternion
    (send _quaternion :serialization-length)
    ;; yesense_imu/YesenseImuEulerAngle _eulerAngle
    (send _eulerAngle :serialization-length)
    ;; yesense_imu/YesenseImuLocation _location
    (send _location :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _sample_timestamp
       (write-long _sample_timestamp s)
     ;; uint32 _sync_timestamp
       (write-long _sync_timestamp s)
     ;; geometry_msgs/Accel _accel
       (send _accel :serialize s)
     ;; yesense_imu/YesenseImuQuaternion _quaternion
       (send _quaternion :serialize s)
     ;; yesense_imu/YesenseImuEulerAngle _eulerAngle
       (send _eulerAngle :serialize s)
     ;; yesense_imu/YesenseImuLocation _location
       (send _location :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _temperature
     (setq _temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _sample_timestamp
     (setq _sample_timestamp (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _sync_timestamp
     (setq _sync_timestamp (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Accel _accel
     (send _accel :deserialize buf ptr-) (incf ptr- (send _accel :serialization-length))
   ;; yesense_imu/YesenseImuQuaternion _quaternion
     (send _quaternion :deserialize buf ptr-) (incf ptr- (send _quaternion :serialization-length))
   ;; yesense_imu/YesenseImuEulerAngle _eulerAngle
     (send _eulerAngle :deserialize buf ptr-) (incf ptr- (send _eulerAngle :serialization-length))
   ;; yesense_imu/YesenseImuLocation _location
     (send _location :deserialize buf ptr-) (incf ptr- (send _location :serialization-length))
   ;;
   self)
  )

(setf (get yesense_imu::YesenseImuSensorData :md5sum-) "6d4626fa769075113f501bc181b31122")
(setf (get yesense_imu::YesenseImuSensorData :datatype-) "yesense_imu/YesenseImuSensorData")
(setf (get yesense_imu::YesenseImuSensorData :definition-)
      "float32 temperature
uint32 sample_timestamp
uint32 sync_timestamp
geometry_msgs/Accel accel
yesense_imu/YesenseImuQuaternion quaternion
yesense_imu/YesenseImuEulerAngle eulerAngle
yesense_imu/YesenseImuLocation location
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: yesense_imu/YesenseImuQuaternion
float32 q0
float32 q1
float32 q2
float32 q3

================================================================================
MSG: yesense_imu/YesenseImuEulerAngle
float32 roll
float32 pitch
float32 yaw

================================================================================
MSG: yesense_imu/YesenseImuLocation
float64 longtidue
float64 latitude
float32 altidue
")



(provide :yesense_imu/YesenseImuSensorData "6d4626fa769075113f501bc181b31122")


