; Auto-generated. Do not edit!


(cl:in-package yesense_imu-msg)


;//! \htmlinclude YesenseImuSlaveGnssData.msg.html

(cl:defclass <YesenseImuSlaveGnssData> (roslisp-msg-protocol:ros-message)
  ((dual_ant_yaw
    :reader dual_ant_yaw
    :initarg :dual_ant_yaw
    :type cl:float
    :initform 0.0)
   (dual_ant_yaw_error
    :reader dual_ant_yaw_error
    :initarg :dual_ant_yaw_error
    :type cl:float
    :initform 0.0)
   (dual_ant_baseline_len
    :reader dual_ant_baseline_len
    :initarg :dual_ant_baseline_len
    :type cl:float
    :initform 0.0))
)

(cl:defclass YesenseImuSlaveGnssData (<YesenseImuSlaveGnssData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YesenseImuSlaveGnssData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YesenseImuSlaveGnssData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yesense_imu-msg:<YesenseImuSlaveGnssData> is deprecated: use yesense_imu-msg:YesenseImuSlaveGnssData instead.")))

(cl:ensure-generic-function 'dual_ant_yaw-val :lambda-list '(m))
(cl:defmethod dual_ant_yaw-val ((m <YesenseImuSlaveGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:dual_ant_yaw-val is deprecated.  Use yesense_imu-msg:dual_ant_yaw instead.")
  (dual_ant_yaw m))

(cl:ensure-generic-function 'dual_ant_yaw_error-val :lambda-list '(m))
(cl:defmethod dual_ant_yaw_error-val ((m <YesenseImuSlaveGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:dual_ant_yaw_error-val is deprecated.  Use yesense_imu-msg:dual_ant_yaw_error instead.")
  (dual_ant_yaw_error m))

(cl:ensure-generic-function 'dual_ant_baseline_len-val :lambda-list '(m))
(cl:defmethod dual_ant_baseline_len-val ((m <YesenseImuSlaveGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:dual_ant_baseline_len-val is deprecated.  Use yesense_imu-msg:dual_ant_baseline_len instead.")
  (dual_ant_baseline_len m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YesenseImuSlaveGnssData>) ostream)
  "Serializes a message object of type '<YesenseImuSlaveGnssData>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dual_ant_yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dual_ant_yaw_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dual_ant_baseline_len))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YesenseImuSlaveGnssData>) istream)
  "Deserializes a message object of type '<YesenseImuSlaveGnssData>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dual_ant_yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dual_ant_yaw_error) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dual_ant_baseline_len) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YesenseImuSlaveGnssData>)))
  "Returns string type for a message object of type '<YesenseImuSlaveGnssData>"
  "yesense_imu/YesenseImuSlaveGnssData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YesenseImuSlaveGnssData)))
  "Returns string type for a message object of type 'YesenseImuSlaveGnssData"
  "yesense_imu/YesenseImuSlaveGnssData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YesenseImuSlaveGnssData>)))
  "Returns md5sum for a message object of type '<YesenseImuSlaveGnssData>"
  "8343f347c57f056bf546331417a9a305")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YesenseImuSlaveGnssData)))
  "Returns md5sum for a message object of type 'YesenseImuSlaveGnssData"
  "8343f347c57f056bf546331417a9a305")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YesenseImuSlaveGnssData>)))
  "Returns full string definition for message of type '<YesenseImuSlaveGnssData>"
  (cl:format cl:nil "float32 dual_ant_yaw~%float32 dual_ant_yaw_error~%float32 dual_ant_baseline_len~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YesenseImuSlaveGnssData)))
  "Returns full string definition for message of type 'YesenseImuSlaveGnssData"
  (cl:format cl:nil "float32 dual_ant_yaw~%float32 dual_ant_yaw_error~%float32 dual_ant_baseline_len~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YesenseImuSlaveGnssData>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YesenseImuSlaveGnssData>))
  "Converts a ROS message object to a list"
  (cl:list 'YesenseImuSlaveGnssData
    (cl:cons ':dual_ant_yaw (dual_ant_yaw msg))
    (cl:cons ':dual_ant_yaw_error (dual_ant_yaw_error msg))
    (cl:cons ':dual_ant_baseline_len (dual_ant_baseline_len msg))
))
