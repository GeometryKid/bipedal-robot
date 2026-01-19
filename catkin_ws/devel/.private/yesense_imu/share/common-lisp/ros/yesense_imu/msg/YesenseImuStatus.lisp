; Auto-generated. Do not edit!


(cl:in-package yesense_imu-msg)


;//! \htmlinclude YesenseImuStatus.msg.html

(cl:defclass <YesenseImuStatus> (roslisp-msg-protocol:ros-message)
  ((fusion_status
    :reader fusion_status
    :initarg :fusion_status
    :type cl:fixnum
    :initform 0)
   (gnss_status
    :reader gnss_status
    :initarg :gnss_status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass YesenseImuStatus (<YesenseImuStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YesenseImuStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YesenseImuStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yesense_imu-msg:<YesenseImuStatus> is deprecated: use yesense_imu-msg:YesenseImuStatus instead.")))

(cl:ensure-generic-function 'fusion_status-val :lambda-list '(m))
(cl:defmethod fusion_status-val ((m <YesenseImuStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:fusion_status-val is deprecated.  Use yesense_imu-msg:fusion_status instead.")
  (fusion_status m))

(cl:ensure-generic-function 'gnss_status-val :lambda-list '(m))
(cl:defmethod gnss_status-val ((m <YesenseImuStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:gnss_status-val is deprecated.  Use yesense_imu-msg:gnss_status instead.")
  (gnss_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YesenseImuStatus>) ostream)
  "Serializes a message object of type '<YesenseImuStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fusion_status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gnss_status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YesenseImuStatus>) istream)
  "Deserializes a message object of type '<YesenseImuStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fusion_status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gnss_status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YesenseImuStatus>)))
  "Returns string type for a message object of type '<YesenseImuStatus>"
  "yesense_imu/YesenseImuStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YesenseImuStatus)))
  "Returns string type for a message object of type 'YesenseImuStatus"
  "yesense_imu/YesenseImuStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YesenseImuStatus>)))
  "Returns md5sum for a message object of type '<YesenseImuStatus>"
  "ce0abe5eeaff955681f09594f0484f51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YesenseImuStatus)))
  "Returns md5sum for a message object of type 'YesenseImuStatus"
  "ce0abe5eeaff955681f09594f0484f51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YesenseImuStatus>)))
  "Returns full string definition for message of type '<YesenseImuStatus>"
  (cl:format cl:nil "uint8 fusion_status~%uint8 gnss_status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YesenseImuStatus)))
  "Returns full string definition for message of type 'YesenseImuStatus"
  (cl:format cl:nil "uint8 fusion_status~%uint8 gnss_status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YesenseImuStatus>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YesenseImuStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'YesenseImuStatus
    (cl:cons ':fusion_status (fusion_status msg))
    (cl:cons ':gnss_status (gnss_status msg))
))
