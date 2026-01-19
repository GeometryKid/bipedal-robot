; Auto-generated. Do not edit!


(cl:in-package yesense_imu-msg)


;//! \htmlinclude YesenseIMUSetting.msg.html

(cl:defclass <YesenseIMUSetting> (roslisp-msg-protocol:ros-message)
  ((class
    :reader class
    :initarg :class
    :type cl:fixnum
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (length
    :reader length
    :initarg :length
    :type cl:fixnum
    :initform 0)
   (msg
    :reader msg
    :initarg :msg
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass YesenseIMUSetting (<YesenseIMUSetting>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YesenseIMUSetting>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YesenseIMUSetting)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yesense_imu-msg:<YesenseIMUSetting> is deprecated: use yesense_imu-msg:YesenseIMUSetting instead.")))

(cl:ensure-generic-function 'class-val :lambda-list '(m))
(cl:defmethod class-val ((m <YesenseIMUSetting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:class-val is deprecated.  Use yesense_imu-msg:class instead.")
  (class m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <YesenseIMUSetting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:id-val is deprecated.  Use yesense_imu-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <YesenseIMUSetting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:length-val is deprecated.  Use yesense_imu-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'msg-val :lambda-list '(m))
(cl:defmethod msg-val ((m <YesenseIMUSetting>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:msg-val is deprecated.  Use yesense_imu-msg:msg instead.")
  (msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YesenseIMUSetting>) ostream)
  "Serializes a message object of type '<YesenseIMUSetting>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'class)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'msg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'msg))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YesenseIMUSetting>) istream)
  "Deserializes a message object of type '<YesenseIMUSetting>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'class)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'length)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'msg) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'msg)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YesenseIMUSetting>)))
  "Returns string type for a message object of type '<YesenseIMUSetting>"
  "yesense_imu/YesenseIMUSetting")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YesenseIMUSetting)))
  "Returns string type for a message object of type 'YesenseIMUSetting"
  "yesense_imu/YesenseIMUSetting")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YesenseIMUSetting>)))
  "Returns md5sum for a message object of type '<YesenseIMUSetting>"
  "7e7a91ef26451a647daf3866c66ac74c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YesenseIMUSetting)))
  "Returns md5sum for a message object of type 'YesenseIMUSetting"
  "7e7a91ef26451a647daf3866c66ac74c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YesenseIMUSetting>)))
  "Returns full string definition for message of type '<YesenseIMUSetting>"
  (cl:format cl:nil "#uint8 CLASS_PRODUCTION_INFO   = 0~%#uint8 CLASS_BAUDRATE          = 2~%#uint8 CLASS_OUTPUT_FREEQUENCY = 3~%#uint8 CLASS_OUTPUT_CONTENT    = 4~%#uint8 CLASS_STANDARD_PARAM    = 5~%#uint8 CLASS_MODE              = 77   #0x4D~%#uint8 CLASS_NMEA_OUTPUT       = 78   #0x4E~%~%#uint8 ID_REQUEST              = 0~%#uint8 ID_MEMERY               = 1~%#uint8 ID_FLASH                = 2~%~%uint8 class~%uint8 id~%uint8 length~%uint8[] msg~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YesenseIMUSetting)))
  "Returns full string definition for message of type 'YesenseIMUSetting"
  (cl:format cl:nil "#uint8 CLASS_PRODUCTION_INFO   = 0~%#uint8 CLASS_BAUDRATE          = 2~%#uint8 CLASS_OUTPUT_FREEQUENCY = 3~%#uint8 CLASS_OUTPUT_CONTENT    = 4~%#uint8 CLASS_STANDARD_PARAM    = 5~%#uint8 CLASS_MODE              = 77   #0x4D~%#uint8 CLASS_NMEA_OUTPUT       = 78   #0x4E~%~%#uint8 ID_REQUEST              = 0~%#uint8 ID_MEMERY               = 1~%#uint8 ID_FLASH                = 2~%~%uint8 class~%uint8 id~%uint8 length~%uint8[] msg~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YesenseIMUSetting>))
  (cl:+ 0
     1
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'msg) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YesenseIMUSetting>))
  "Converts a ROS message object to a list"
  (cl:list 'YesenseIMUSetting
    (cl:cons ':class (class msg))
    (cl:cons ':id (id msg))
    (cl:cons ':length (length msg))
    (cl:cons ':msg (msg msg))
))
