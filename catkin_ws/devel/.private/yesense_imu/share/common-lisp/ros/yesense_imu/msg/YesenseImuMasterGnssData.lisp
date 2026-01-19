; Auto-generated. Do not edit!


(cl:in-package yesense_imu-msg)


;//! \htmlinclude YesenseImuMasterGnssData.msg.html

(cl:defclass <YesenseImuMasterGnssData> (roslisp-msg-protocol:ros-message)
  ((utc_time
    :reader utc_time
    :initarg :utc_time
    :type yesense_imu-msg:YesenseImuUtcTime
    :initform (cl:make-instance 'yesense_imu-msg:YesenseImuUtcTime))
   (location
    :reader location
    :initarg :location
    :type yesense_imu-msg:YesenseImuLocation
    :initform (cl:make-instance 'yesense_imu-msg:YesenseImuLocation))
   (location_error
    :reader location_error
    :initarg :location_error
    :type yesense_imu-msg:YesenseImuLocation
    :initform (cl:make-instance 'yesense_imu-msg:YesenseImuLocation))
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0)
   (star_cnt
    :reader star_cnt
    :initarg :star_cnt
    :type cl:fixnum
    :initform 0)
   (p_dop
    :reader p_dop
    :initarg :p_dop
    :type cl:float
    :initform 0.0)
   (site_id
    :reader site_id
    :initarg :site_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass YesenseImuMasterGnssData (<YesenseImuMasterGnssData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YesenseImuMasterGnssData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YesenseImuMasterGnssData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yesense_imu-msg:<YesenseImuMasterGnssData> is deprecated: use yesense_imu-msg:YesenseImuMasterGnssData instead.")))

(cl:ensure-generic-function 'utc_time-val :lambda-list '(m))
(cl:defmethod utc_time-val ((m <YesenseImuMasterGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:utc_time-val is deprecated.  Use yesense_imu-msg:utc_time instead.")
  (utc_time m))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <YesenseImuMasterGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:location-val is deprecated.  Use yesense_imu-msg:location instead.")
  (location m))

(cl:ensure-generic-function 'location_error-val :lambda-list '(m))
(cl:defmethod location_error-val ((m <YesenseImuMasterGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:location_error-val is deprecated.  Use yesense_imu-msg:location_error instead.")
  (location_error m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <YesenseImuMasterGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:speed-val is deprecated.  Use yesense_imu-msg:speed instead.")
  (speed m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <YesenseImuMasterGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:yaw-val is deprecated.  Use yesense_imu-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <YesenseImuMasterGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:status-val is deprecated.  Use yesense_imu-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'star_cnt-val :lambda-list '(m))
(cl:defmethod star_cnt-val ((m <YesenseImuMasterGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:star_cnt-val is deprecated.  Use yesense_imu-msg:star_cnt instead.")
  (star_cnt m))

(cl:ensure-generic-function 'p_dop-val :lambda-list '(m))
(cl:defmethod p_dop-val ((m <YesenseImuMasterGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:p_dop-val is deprecated.  Use yesense_imu-msg:p_dop instead.")
  (p_dop m))

(cl:ensure-generic-function 'site_id-val :lambda-list '(m))
(cl:defmethod site_id-val ((m <YesenseImuMasterGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:site_id-val is deprecated.  Use yesense_imu-msg:site_id instead.")
  (site_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YesenseImuMasterGnssData>) ostream)
  "Serializes a message object of type '<YesenseImuMasterGnssData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'utc_time) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'location) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'location_error) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'star_cnt)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p_dop))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'site_id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YesenseImuMasterGnssData>) istream)
  "Deserializes a message object of type '<YesenseImuMasterGnssData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'utc_time) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'location) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'location_error) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'star_cnt)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p_dop) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'site_id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YesenseImuMasterGnssData>)))
  "Returns string type for a message object of type '<YesenseImuMasterGnssData>"
  "yesense_imu/YesenseImuMasterGnssData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YesenseImuMasterGnssData)))
  "Returns string type for a message object of type 'YesenseImuMasterGnssData"
  "yesense_imu/YesenseImuMasterGnssData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YesenseImuMasterGnssData>)))
  "Returns md5sum for a message object of type '<YesenseImuMasterGnssData>"
  "707ba72b8fe698890901dc5ceaf55863")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YesenseImuMasterGnssData)))
  "Returns md5sum for a message object of type 'YesenseImuMasterGnssData"
  "707ba72b8fe698890901dc5ceaf55863")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YesenseImuMasterGnssData>)))
  "Returns full string definition for message of type '<YesenseImuMasterGnssData>"
  (cl:format cl:nil "yesense_imu/YesenseImuUtcTime utc_time~%yesense_imu/YesenseImuLocation location~%yesense_imu/YesenseImuLocation location_error~%float32 speed~%float32 yaw~%uint8 status~%uint8 star_cnt~%float32 p_dop~%uint8 site_id~%================================================================================~%MSG: yesense_imu/YesenseImuUtcTime~%uint16 year~%uint8 month~%uint8 date~%uint8 hour~%uint8 min~%uint8 sec~%uint16 ms~%================================================================================~%MSG: yesense_imu/YesenseImuLocation~%float64 longtidue~%float64 latitude~%float32 altidue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YesenseImuMasterGnssData)))
  "Returns full string definition for message of type 'YesenseImuMasterGnssData"
  (cl:format cl:nil "yesense_imu/YesenseImuUtcTime utc_time~%yesense_imu/YesenseImuLocation location~%yesense_imu/YesenseImuLocation location_error~%float32 speed~%float32 yaw~%uint8 status~%uint8 star_cnt~%float32 p_dop~%uint8 site_id~%================================================================================~%MSG: yesense_imu/YesenseImuUtcTime~%uint16 year~%uint8 month~%uint8 date~%uint8 hour~%uint8 min~%uint8 sec~%uint16 ms~%================================================================================~%MSG: yesense_imu/YesenseImuLocation~%float64 longtidue~%float64 latitude~%float32 altidue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YesenseImuMasterGnssData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'utc_time))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'location))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'location_error))
     4
     4
     1
     1
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YesenseImuMasterGnssData>))
  "Converts a ROS message object to a list"
  (cl:list 'YesenseImuMasterGnssData
    (cl:cons ':utc_time (utc_time msg))
    (cl:cons ':location (location msg))
    (cl:cons ':location_error (location_error msg))
    (cl:cons ':speed (speed msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':status (status msg))
    (cl:cons ':star_cnt (star_cnt msg))
    (cl:cons ':p_dop (p_dop msg))
    (cl:cons ':site_id (site_id msg))
))
