; Auto-generated. Do not edit!


(cl:in-package id_data_msgs-msg)


;//! \htmlinclude forceData.msg.html

(cl:defclass <forceData> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:integer)
   :initform (cl:make-array 64 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass forceData (<forceData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <forceData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'forceData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name id_data_msgs-msg:<forceData> is deprecated: use id_data_msgs-msg:forceData instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <forceData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader id_data_msgs-msg:id-val is deprecated.  Use id_data_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <forceData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader id_data_msgs-msg:data-val is deprecated.  Use id_data_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <forceData>) ostream)
  "Serializes a message object of type '<forceData>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <forceData>) istream)
  "Deserializes a message object of type '<forceData>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 64))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 64)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<forceData>)))
  "Returns string type for a message object of type '<forceData>"
  "id_data_msgs/forceData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'forceData)))
  "Returns string type for a message object of type 'forceData"
  "id_data_msgs/forceData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<forceData>)))
  "Returns md5sum for a message object of type '<forceData>"
  "f33eeef6343d38db323c7d076efb464e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'forceData)))
  "Returns md5sum for a message object of type 'forceData"
  "f33eeef6343d38db323c7d076efb464e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<forceData>)))
  "Returns full string definition for message of type '<forceData>"
  (cl:format cl:nil "int32 id~%int32[64] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'forceData)))
  "Returns full string definition for message of type 'forceData"
  (cl:format cl:nil "int32 id~%int32[64] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <forceData>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <forceData>))
  "Converts a ROS message object to a list"
  (cl:list 'forceData
    (cl:cons ':id (id msg))
    (cl:cons ':data (data msg))
))
