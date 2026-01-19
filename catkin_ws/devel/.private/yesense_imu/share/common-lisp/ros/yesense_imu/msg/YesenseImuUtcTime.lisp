; Auto-generated. Do not edit!


(cl:in-package yesense_imu-msg)


;//! \htmlinclude YesenseImuUtcTime.msg.html

(cl:defclass <YesenseImuUtcTime> (roslisp-msg-protocol:ros-message)
  ((year
    :reader year
    :initarg :year
    :type cl:fixnum
    :initform 0)
   (month
    :reader month
    :initarg :month
    :type cl:fixnum
    :initform 0)
   (date
    :reader date
    :initarg :date
    :type cl:fixnum
    :initform 0)
   (hour
    :reader hour
    :initarg :hour
    :type cl:fixnum
    :initform 0)
   (min
    :reader min
    :initarg :min
    :type cl:fixnum
    :initform 0)
   (sec
    :reader sec
    :initarg :sec
    :type cl:fixnum
    :initform 0)
   (ms
    :reader ms
    :initarg :ms
    :type cl:fixnum
    :initform 0))
)

(cl:defclass YesenseImuUtcTime (<YesenseImuUtcTime>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YesenseImuUtcTime>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YesenseImuUtcTime)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yesense_imu-msg:<YesenseImuUtcTime> is deprecated: use yesense_imu-msg:YesenseImuUtcTime instead.")))

(cl:ensure-generic-function 'year-val :lambda-list '(m))
(cl:defmethod year-val ((m <YesenseImuUtcTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:year-val is deprecated.  Use yesense_imu-msg:year instead.")
  (year m))

(cl:ensure-generic-function 'month-val :lambda-list '(m))
(cl:defmethod month-val ((m <YesenseImuUtcTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:month-val is deprecated.  Use yesense_imu-msg:month instead.")
  (month m))

(cl:ensure-generic-function 'date-val :lambda-list '(m))
(cl:defmethod date-val ((m <YesenseImuUtcTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:date-val is deprecated.  Use yesense_imu-msg:date instead.")
  (date m))

(cl:ensure-generic-function 'hour-val :lambda-list '(m))
(cl:defmethod hour-val ((m <YesenseImuUtcTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:hour-val is deprecated.  Use yesense_imu-msg:hour instead.")
  (hour m))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <YesenseImuUtcTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:min-val is deprecated.  Use yesense_imu-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'sec-val :lambda-list '(m))
(cl:defmethod sec-val ((m <YesenseImuUtcTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:sec-val is deprecated.  Use yesense_imu-msg:sec instead.")
  (sec m))

(cl:ensure-generic-function 'ms-val :lambda-list '(m))
(cl:defmethod ms-val ((m <YesenseImuUtcTime>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:ms-val is deprecated.  Use yesense_imu-msg:ms instead.")
  (ms m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YesenseImuUtcTime>) ostream)
  "Serializes a message object of type '<YesenseImuUtcTime>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'year)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'month)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'date)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hour)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sec)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ms)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ms)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YesenseImuUtcTime>) istream)
  "Deserializes a message object of type '<YesenseImuUtcTime>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'year)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'month)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'date)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hour)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'min)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sec)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ms)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'ms)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YesenseImuUtcTime>)))
  "Returns string type for a message object of type '<YesenseImuUtcTime>"
  "yesense_imu/YesenseImuUtcTime")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YesenseImuUtcTime)))
  "Returns string type for a message object of type 'YesenseImuUtcTime"
  "yesense_imu/YesenseImuUtcTime")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YesenseImuUtcTime>)))
  "Returns md5sum for a message object of type '<YesenseImuUtcTime>"
  "9ef427732f567725a6400541b0a0bb7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YesenseImuUtcTime)))
  "Returns md5sum for a message object of type 'YesenseImuUtcTime"
  "9ef427732f567725a6400541b0a0bb7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YesenseImuUtcTime>)))
  "Returns full string definition for message of type '<YesenseImuUtcTime>"
  (cl:format cl:nil "uint16 year~%uint8 month~%uint8 date~%uint8 hour~%uint8 min~%uint8 sec~%uint16 ms~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YesenseImuUtcTime)))
  "Returns full string definition for message of type 'YesenseImuUtcTime"
  (cl:format cl:nil "uint16 year~%uint8 month~%uint8 date~%uint8 hour~%uint8 min~%uint8 sec~%uint16 ms~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YesenseImuUtcTime>))
  (cl:+ 0
     2
     1
     1
     1
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YesenseImuUtcTime>))
  "Converts a ROS message object to a list"
  (cl:list 'YesenseImuUtcTime
    (cl:cons ':year (year msg))
    (cl:cons ':month (month msg))
    (cl:cons ':date (date msg))
    (cl:cons ':hour (hour msg))
    (cl:cons ':min (min msg))
    (cl:cons ':sec (sec msg))
    (cl:cons ':ms (ms msg))
))
