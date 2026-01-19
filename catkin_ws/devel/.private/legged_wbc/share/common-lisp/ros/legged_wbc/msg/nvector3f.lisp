; Auto-generated. Do not edit!


(cl:in-package legged_wbc-msg)


;//! \htmlinclude nvector3f.msg.html

(cl:defclass <nvector3f> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector legged_wbc-msg:vector3f)
   :initform (cl:make-array 0 :element-type 'legged_wbc-msg:vector3f :initial-element (cl:make-instance 'legged_wbc-msg:vector3f))))
)

(cl:defclass nvector3f (<nvector3f>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <nvector3f>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'nvector3f)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name legged_wbc-msg:<nvector3f> is deprecated: use legged_wbc-msg:nvector3f instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <nvector3f>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader legged_wbc-msg:data-val is deprecated.  Use legged_wbc-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <nvector3f>) ostream)
  "Serializes a message object of type '<nvector3f>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <nvector3f>) istream)
  "Deserializes a message object of type '<nvector3f>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'legged_wbc-msg:vector3f))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<nvector3f>)))
  "Returns string type for a message object of type '<nvector3f>"
  "legged_wbc/nvector3f")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'nvector3f)))
  "Returns string type for a message object of type 'nvector3f"
  "legged_wbc/nvector3f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<nvector3f>)))
  "Returns md5sum for a message object of type '<nvector3f>"
  "390cbff96d168abc35f84f5ed0728288")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'nvector3f)))
  "Returns md5sum for a message object of type 'nvector3f"
  "390cbff96d168abc35f84f5ed0728288")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<nvector3f>)))
  "Returns full string definition for message of type '<nvector3f>"
  (cl:format cl:nil "vector3f[] data~%================================================================================~%MSG: legged_wbc/vector3f~%float64 x ~%float64 y ~%float64 z ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'nvector3f)))
  "Returns full string definition for message of type 'nvector3f"
  (cl:format cl:nil "vector3f[] data~%================================================================================~%MSG: legged_wbc/vector3f~%float64 x ~%float64 y ~%float64 z ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <nvector3f>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <nvector3f>))
  "Converts a ROS message object to a list"
  (cl:list 'nvector3f
    (cl:cons ':data (data msg))
))
