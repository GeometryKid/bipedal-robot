;; Auto-generated. Do not edit!


(when (boundp 'legged_wbc::nvector3f)
  (if (not (find-package "LEGGED_WBC"))
    (make-package "LEGGED_WBC"))
  (shadow 'nvector3f (find-package "LEGGED_WBC")))
(unless (find-package "LEGGED_WBC::NVECTOR3F")
  (make-package "LEGGED_WBC::NVECTOR3F"))

(in-package "ROS")
;;//! \htmlinclude nvector3f.msg.html


(defclass legged_wbc::nvector3f
  :super ros::object
  :slots (_data ))

(defmethod legged_wbc::nvector3f
  (:init
   (&key
    ((:data __data) ())
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; legged_wbc/vector3f[] _data
    (apply #'+ (send-all _data :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; legged_wbc/vector3f[] _data
     (write-long (length _data) s)
     (dolist (elem _data)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; legged_wbc/vector3f[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (let (r) (dotimes (i n) (push (instance legged_wbc::vector3f :init) r)) r))
     (dolist (elem- _data)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get legged_wbc::nvector3f :md5sum-) "390cbff96d168abc35f84f5ed0728288")
(setf (get legged_wbc::nvector3f :datatype-) "legged_wbc/nvector3f")
(setf (get legged_wbc::nvector3f :definition-)
      "vector3f[] data
================================================================================
MSG: legged_wbc/vector3f
float64 x 
float64 y 
float64 z 

")



(provide :legged_wbc/nvector3f "390cbff96d168abc35f84f5ed0728288")


