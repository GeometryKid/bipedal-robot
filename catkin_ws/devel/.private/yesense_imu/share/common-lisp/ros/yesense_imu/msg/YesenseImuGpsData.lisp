; Auto-generated. Do not edit!


(cl:in-package yesense_imu-msg)


;//! \htmlinclude YesenseImuGpsData.msg.html

(cl:defclass <YesenseImuGpsData> (roslisp-msg-protocol:ros-message)
  ((raw_data
    :reader raw_data
    :initarg :raw_data
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass YesenseImuGpsData (<YesenseImuGpsData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <YesenseImuGpsData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'YesenseImuGpsData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yesense_imu-msg:<YesenseImuGpsData> is deprecated: use yesense_imu-msg:YesenseImuGpsData instead.")))

(cl:ensure-generic-function 'raw_data-val :lambda-list '(m))
(cl:defmethod raw_data-val ((m <YesenseImuGpsData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yesense_imu-msg:raw_data-val is deprecated.  Use yesense_imu-msg:raw_data instead.")
  (raw_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <YesenseImuGpsData>) ostream)
  "Serializes a message object of type '<YesenseImuGpsData>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'raw_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'raw_data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <YesenseImuGpsData>) istream)
  "Deserializes a message object of type '<YesenseImuGpsData>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'raw_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'raw_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<YesenseImuGpsData>)))
  "Returns string type for a message object of type '<YesenseImuGpsData>"
  "yesense_imu/YesenseImuGpsData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'YesenseImuGpsData)))
  "Returns string type for a message object of type 'YesenseImuGpsData"
  "yesense_imu/YesenseImuGpsData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<YesenseImuGpsData>)))
  "Returns md5sum for a message object of type '<YesenseImuGpsData>"
  "a93dac2a6e802ac9f45a79f2b894d63d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'YesenseImuGpsData)))
  "Returns md5sum for a message object of type 'YesenseImuGpsData"
  "a93dac2a6e802ac9f45a79f2b894d63d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<YesenseImuGpsData>)))
  "Returns full string definition for message of type '<YesenseImuGpsData>"
  (cl:format cl:nil "string[] raw_data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'YesenseImuGpsData)))
  "Returns full string definition for message of type 'YesenseImuGpsData"
  (cl:format cl:nil "string[] raw_data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <YesenseImuGpsData>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'raw_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <YesenseImuGpsData>))
  "Converts a ROS message object to a list"
  (cl:list 'YesenseImuGpsData
    (cl:cons ':raw_data (raw_data msg))
))
