; Auto-generated. Do not edit!


(cl:in-package id_data_msgs-msg)


;//! \htmlinclude ID_Data.msg.html

(cl:defclass <ID_Data> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:integer)
   :initform (cl:make-array 8 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass ID_Data (<ID_Data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ID_Data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ID_Data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name id_data_msgs-msg:<ID_Data> is deprecated: use id_data_msgs-msg:ID_Data instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <ID_Data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader id_data_msgs-msg:id-val is deprecated.  Use id_data_msgs-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <ID_Data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader id_data_msgs-msg:data-val is deprecated.  Use id_data_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ID_Data>) ostream)
  "Serializes a message object of type '<ID_Data>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ID_Data>) istream)
  "Deserializes a message object of type '<ID_Data>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 8))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 8)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ID_Data>)))
  "Returns string type for a message object of type '<ID_Data>"
  "id_data_msgs/ID_Data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ID_Data)))
  "Returns string type for a message object of type 'ID_Data"
  "id_data_msgs/ID_Data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ID_Data>)))
  "Returns md5sum for a message object of type '<ID_Data>"
  "17268f86caecf51769ca5729d0bb14c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ID_Data)))
  "Returns md5sum for a message object of type 'ID_Data"
  "17268f86caecf51769ca5729d0bb14c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ID_Data>)))
  "Returns full string definition for message of type '<ID_Data>"
  (cl:format cl:nil "int32 id~%int32[8] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ID_Data)))
  "Returns full string definition for message of type 'ID_Data"
  (cl:format cl:nil "int32 id~%int32[8] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ID_Data>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ID_Data>))
  "Converts a ROS message object to a list"
  (cl:list 'ID_Data
    (cl:cons ':id (id msg))
    (cl:cons ':data (data msg))
))
