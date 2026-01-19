; Auto-generated. Do not edit!


(cl:in-package yesense_imu-msg)


;//! \htmlinclude YesenseImuGnssData.msg.html

(cl:defclass <YesenseImuGnssData> (roslisp-msg-protocol:ros-message)
  ((master
    :reader master
    :initarg :master
    :type yesense_imu-msg:YesenseImuMasterGnssData
    :initform (cl:make-instance 'yesense_imu-msg:YesenseImuMasterGnssData))
   (slave
    :reader slave
    :initarg :slave
    :type yesense_imu-msg:YesenseImuSlaveGnssData
    :initform (cl:make-instance 'yesense_imu-msg:YesenseImuSlaveGnssData)))
)

(cl:defclass YesenseImuGnssData (<YesenseImuGnssData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YesenseImuGnssData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YesenseImuGnssData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yesense_imu-msg:<YesenseImuGnssData> is deprecated: use yesense_imu-msg:YesenseImuGnssData instead.")))

(cl:ensure-generic-function 'master-val :lambda-list '(m))
(cl:defmethod master-val ((m <YesenseImuGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:master-val is deprecated.  Use yesense_imu-msg:master instead.")
  (master m))

(cl:ensure-generic-function 'slave-val :lambda-list '(m))
(cl:defmethod slave-val ((m <YesenseImuGnssData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:slave-val is deprecated.  Use yesense_imu-msg:slave instead.")
  (slave m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YesenseImuGnssData>) ostream)
  "Serializes a message object of type '<YesenseImuGnssData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'master) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'slave) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YesenseImuGnssData>) istream)
  "Deserializes a message object of type '<YesenseImuGnssData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'master) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'slave) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YesenseImuGnssData>)))
  "Returns string type for a message object of type '<YesenseImuGnssData>"
  "yesense_imu/YesenseImuGnssData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YesenseImuGnssData)))
  "Returns string type for a message object of type 'YesenseImuGnssData"
  "yesense_imu/YesenseImuGnssData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YesenseImuGnssData>)))
  "Returns md5sum for a message object of type '<YesenseImuGnssData>"
  "4ae993c6fa242f94866ceb31ffc6e7d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YesenseImuGnssData)))
  "Returns md5sum for a message object of type 'YesenseImuGnssData"
  "4ae993c6fa242f94866ceb31ffc6e7d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YesenseImuGnssData>)))
  "Returns full string definition for message of type '<YesenseImuGnssData>"
  (cl:format cl:nil "yesense_imu/YesenseImuMasterGnssData master~%yesense_imu/YesenseImuSlaveGnssData slave~%================================================================================~%MSG: yesense_imu/YesenseImuMasterGnssData~%yesense_imu/YesenseImuUtcTime utc_time~%yesense_imu/YesenseImuLocation location~%yesense_imu/YesenseImuLocation location_error~%float32 speed~%float32 yaw~%uint8 status~%uint8 star_cnt~%float32 p_dop~%uint8 site_id~%================================================================================~%MSG: yesense_imu/YesenseImuUtcTime~%uint16 year~%uint8 month~%uint8 date~%uint8 hour~%uint8 min~%uint8 sec~%uint16 ms~%================================================================================~%MSG: yesense_imu/YesenseImuLocation~%float64 longtidue~%float64 latitude~%float32 altidue~%================================================================================~%MSG: yesense_imu/YesenseImuSlaveGnssData~%float32 dual_ant_yaw~%float32 dual_ant_yaw_error~%float32 dual_ant_baseline_len~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YesenseImuGnssData)))
  "Returns full string definition for message of type 'YesenseImuGnssData"
  (cl:format cl:nil "yesense_imu/YesenseImuMasterGnssData master~%yesense_imu/YesenseImuSlaveGnssData slave~%================================================================================~%MSG: yesense_imu/YesenseImuMasterGnssData~%yesense_imu/YesenseImuUtcTime utc_time~%yesense_imu/YesenseImuLocation location~%yesense_imu/YesenseImuLocation location_error~%float32 speed~%float32 yaw~%uint8 status~%uint8 star_cnt~%float32 p_dop~%uint8 site_id~%================================================================================~%MSG: yesense_imu/YesenseImuUtcTime~%uint16 year~%uint8 month~%uint8 date~%uint8 hour~%uint8 min~%uint8 sec~%uint16 ms~%================================================================================~%MSG: yesense_imu/YesenseImuLocation~%float64 longtidue~%float64 latitude~%float32 altidue~%================================================================================~%MSG: yesense_imu/YesenseImuSlaveGnssData~%float32 dual_ant_yaw~%float32 dual_ant_yaw_error~%float32 dual_ant_baseline_len~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YesenseImuGnssData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'master))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'slave))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YesenseImuGnssData>))
  "Converts a ROS message object to a list"
  (cl:list 'YesenseImuGnssData
    (cl:cons ':master (master msg))
    (cl:cons ':slave (slave msg))
))
