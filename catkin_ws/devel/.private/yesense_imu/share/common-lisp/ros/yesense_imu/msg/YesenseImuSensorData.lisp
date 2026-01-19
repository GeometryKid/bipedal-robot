; Auto-generated. Do not edit!


(cl:in-package yesense_imu-msg)


;//! \htmlinclude YesenseImuSensorData.msg.html

(cl:defclass <YesenseImuSensorData> (roslisp-msg-protocol:ros-message)
  ((temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (sample_timestamp
    :reader sample_timestamp
    :initarg :sample_timestamp
    :type cl:integer
    :initform 0)
   (sync_timestamp
    :reader sync_timestamp
    :initarg :sync_timestamp
    :type cl:integer
    :initform 0)
   (accel
    :reader accel
    :initarg :accel
    :type geometry_msgs-msg:Accel
    :initform (cl:make-instance 'geometry_msgs-msg:Accel))
   (quaternion
    :reader quaternion
    :initarg :quaternion
    :type yesense_imu-msg:YesenseImuQuaternion
    :initform (cl:make-instance 'yesense_imu-msg:YesenseImuQuaternion))
   (eulerAngle
    :reader eulerAngle
    :initarg :eulerAngle
    :type yesense_imu-msg:YesenseImuEulerAngle
    :initform (cl:make-instance 'yesense_imu-msg:YesenseImuEulerAngle))
   (location
    :reader location
    :initarg :location
    :type yesense_imu-msg:YesenseImuLocation
    :initform (cl:make-instance 'yesense_imu-msg:YesenseImuLocation)))
)

(cl:defclass YesenseImuSensorData (<YesenseImuSensorData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YesenseImuSensorData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YesenseImuSensorData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yesense_imu-msg:<YesenseImuSensorData> is deprecated: use yesense_imu-msg:YesenseImuSensorData instead.")))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <YesenseImuSensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:temperature-val is deprecated.  Use yesense_imu-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'sample_timestamp-val :lambda-list '(m))
(cl:defmethod sample_timestamp-val ((m <YesenseImuSensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:sample_timestamp-val is deprecated.  Use yesense_imu-msg:sample_timestamp instead.")
  (sample_timestamp m))

(cl:ensure-generic-function 'sync_timestamp-val :lambda-list '(m))
(cl:defmethod sync_timestamp-val ((m <YesenseImuSensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:sync_timestamp-val is deprecated.  Use yesense_imu-msg:sync_timestamp instead.")
  (sync_timestamp m))

(cl:ensure-generic-function 'accel-val :lambda-list '(m))
(cl:defmethod accel-val ((m <YesenseImuSensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:accel-val is deprecated.  Use yesense_imu-msg:accel instead.")
  (accel m))

(cl:ensure-generic-function 'quaternion-val :lambda-list '(m))
(cl:defmethod quaternion-val ((m <YesenseImuSensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:quaternion-val is deprecated.  Use yesense_imu-msg:quaternion instead.")
  (quaternion m))

(cl:ensure-generic-function 'eulerAngle-val :lambda-list '(m))
(cl:defmethod eulerAngle-val ((m <YesenseImuSensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:eulerAngle-val is deprecated.  Use yesense_imu-msg:eulerAngle instead.")
  (eulerAngle m))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <YesenseImuSensorData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:location-val is deprecated.  Use yesense_imu-msg:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YesenseImuSensorData>) ostream)
  "Serializes a message object of type '<YesenseImuSensorData>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sample_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sample_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sample_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sample_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sync_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sync_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sync_timestamp)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sync_timestamp)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'quaternion) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'eulerAngle) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'location) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YesenseImuSensorData>) istream)
  "Deserializes a message object of type '<YesenseImuSensorData>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sample_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sample_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sample_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sample_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sync_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sync_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sync_timestamp)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sync_timestamp)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'quaternion) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'eulerAngle) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'location) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YesenseImuSensorData>)))
  "Returns string type for a message object of type '<YesenseImuSensorData>"
  "yesense_imu/YesenseImuSensorData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YesenseImuSensorData)))
  "Returns string type for a message object of type 'YesenseImuSensorData"
  "yesense_imu/YesenseImuSensorData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YesenseImuSensorData>)))
  "Returns md5sum for a message object of type '<YesenseImuSensorData>"
  "6d4626fa769075113f501bc181b31122")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YesenseImuSensorData)))
  "Returns md5sum for a message object of type 'YesenseImuSensorData"
  "6d4626fa769075113f501bc181b31122")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YesenseImuSensorData>)))
  "Returns full string definition for message of type '<YesenseImuSensorData>"
  (cl:format cl:nil "float32 temperature~%uint32 sample_timestamp~%uint32 sync_timestamp~%geometry_msgs/Accel accel~%yesense_imu/YesenseImuQuaternion quaternion~%yesense_imu/YesenseImuEulerAngle eulerAngle~%yesense_imu/YesenseImuLocation location~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: yesense_imu/YesenseImuQuaternion~%float32 q0~%float32 q1~%float32 q2~%float32 q3~%~%================================================================================~%MSG: yesense_imu/YesenseImuEulerAngle~%float32 roll~%float32 pitch~%float32 yaw~%~%================================================================================~%MSG: yesense_imu/YesenseImuLocation~%float64 longtidue~%float64 latitude~%float32 altidue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YesenseImuSensorData)))
  "Returns full string definition for message of type 'YesenseImuSensorData"
  (cl:format cl:nil "float32 temperature~%uint32 sample_timestamp~%uint32 sync_timestamp~%geometry_msgs/Accel accel~%yesense_imu/YesenseImuQuaternion quaternion~%yesense_imu/YesenseImuEulerAngle eulerAngle~%yesense_imu/YesenseImuLocation location~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: yesense_imu/YesenseImuQuaternion~%float32 q0~%float32 q1~%float32 q2~%float32 q3~%~%================================================================================~%MSG: yesense_imu/YesenseImuEulerAngle~%float32 roll~%float32 pitch~%float32 yaw~%~%================================================================================~%MSG: yesense_imu/YesenseImuLocation~%float64 longtidue~%float64 latitude~%float32 altidue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YesenseImuSensorData>))
  (cl:+ 0
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'quaternion))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'eulerAngle))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'location))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YesenseImuSensorData>))
  "Converts a ROS message object to a list"
  (cl:list 'YesenseImuSensorData
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':sample_timestamp (sample_timestamp msg))
    (cl:cons ':sync_timestamp (sync_timestamp msg))
    (cl:cons ':accel (accel msg))
    (cl:cons ':quaternion (quaternion msg))
    (cl:cons ':eulerAngle (eulerAngle msg))
    (cl:cons ':location (location msg))
))
