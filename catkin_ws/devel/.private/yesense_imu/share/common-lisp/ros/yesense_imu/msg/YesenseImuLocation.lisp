; Auto-generated. Do not edit!


(cl:in-package yesense_imu-msg)


;//! \htmlinclude YesenseImuLocation.msg.html

(cl:defclass <YesenseImuLocation> (roslisp-msg-protocol:ros-message)
  ((longtidue
    :reader longtidue
    :initarg :longtidue
    :type cl:float
    :initform 0.0)
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (altidue
    :reader altidue
    :initarg :altidue
    :type cl:float
    :initform 0.0))
)

(cl:defclass YesenseImuLocation (<YesenseImuLocation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YesenseImuLocation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YesenseImuLocation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yesense_imu-msg:<YesenseImuLocation> is deprecated: use yesense_imu-msg:YesenseImuLocation instead.")))

(cl:ensure-generic-function 'longtidue-val :lambda-list '(m))
(cl:defmethod longtidue-val ((m <YesenseImuLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:longtidue-val is deprecated.  Use yesense_imu-msg:longtidue instead.")
  (longtidue m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <YesenseImuLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:latitude-val is deprecated.  Use yesense_imu-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'altidue-val :lambda-list '(m))
(cl:defmethod altidue-val ((m <YesenseImuLocation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:altidue-val is deprecated.  Use yesense_imu-msg:altidue instead.")
  (altidue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YesenseImuLocation>) ostream)
  "Serializes a message object of type '<YesenseImuLocation>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longtidue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altidue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YesenseImuLocation>) istream)
  "Deserializes a message object of type '<YesenseImuLocation>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longtidue) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altidue) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YesenseImuLocation>)))
  "Returns string type for a message object of type '<YesenseImuLocation>"
  "yesense_imu/YesenseImuLocation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YesenseImuLocation)))
  "Returns string type for a message object of type 'YesenseImuLocation"
  "yesense_imu/YesenseImuLocation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YesenseImuLocation>)))
  "Returns md5sum for a message object of type '<YesenseImuLocation>"
  "5cf93867aa740e2448c799a9da22809d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YesenseImuLocation)))
  "Returns md5sum for a message object of type 'YesenseImuLocation"
  "5cf93867aa740e2448c799a9da22809d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YesenseImuLocation>)))
  "Returns full string definition for message of type '<YesenseImuLocation>"
  (cl:format cl:nil "float64 longtidue~%float64 latitude~%float32 altidue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YesenseImuLocation)))
  "Returns full string definition for message of type 'YesenseImuLocation"
  (cl:format cl:nil "float64 longtidue~%float64 latitude~%float32 altidue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YesenseImuLocation>))
  (cl:+ 0
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YesenseImuLocation>))
  "Converts a ROS message object to a list"
  (cl:list 'YesenseImuLocation
    (cl:cons ':longtidue (longtidue msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':altidue (altidue msg))
))
