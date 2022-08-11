; Auto-generated. Do not edit!


(cl:in-package file_server-srv)


;//! \htmlinclude SaveBinaryFile-request.msg.html

(cl:defclass <SaveBinaryFile-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass SaveBinaryFile-request (<SaveBinaryFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveBinaryFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveBinaryFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name file_server-srv:<SaveBinaryFile-request> is deprecated: use file_server-srv:SaveBinaryFile-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SaveBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_server-srv:name-val is deprecated.  Use file_server-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SaveBinaryFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_server-srv:value-val is deprecated.  Use file_server-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveBinaryFile-request>) ostream)
  "Serializes a message object of type '<SaveBinaryFile-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveBinaryFile-request>) istream)
  "Deserializes a message object of type '<SaveBinaryFile-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'value) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'value)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveBinaryFile-request>)))
  "Returns string type for a service object of type '<SaveBinaryFile-request>"
  "file_server/SaveBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveBinaryFile-request)))
  "Returns string type for a service object of type 'SaveBinaryFile-request"
  "file_server/SaveBinaryFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveBinaryFile-request>)))
  "Returns md5sum for a message object of type '<SaveBinaryFile-request>"
  "3121441b66090c806f9864e12420345a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveBinaryFile-request)))
  "Returns md5sum for a message object of type 'SaveBinaryFile-request"
  "3121441b66090c806f9864e12420345a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveBinaryFile-request>)))
  "Returns full string definition for message of type '<SaveBinaryFile-request>"
  (cl:format cl:nil "string name~%uint8[] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveBinaryFile-request)))
  "Returns full string definition for message of type 'SaveBinaryFile-request"
  (cl:format cl:nil "string name~%uint8[] value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveBinaryFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'value) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveBinaryFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveBinaryFile-request
    (cl:cons ':name (name msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SaveBinaryFile-response.msg.html

(cl:defclass <SaveBinaryFile-response> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass SaveBinaryFile-response (<SaveBinaryFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SaveBinaryFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SaveBinaryFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name file_server-srv:<SaveBinaryFile-response> is deprecated: use file_server-srv:SaveBinaryFile-response instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <SaveBinaryFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader file_server-srv:name-val is deprecated.  Use file_server-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SaveBinaryFile-response>) ostream)
  "Serializes a message object of type '<SaveBinaryFile-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SaveBinaryFile-response>) istream)
  "Deserializes a message object of type '<SaveBinaryFile-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SaveBinaryFile-response>)))
  "Returns string type for a service object of type '<SaveBinaryFile-response>"
  "file_server/SaveBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveBinaryFile-response)))
  "Returns string type for a service object of type 'SaveBinaryFile-response"
  "file_server/SaveBinaryFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SaveBinaryFile-response>)))
  "Returns md5sum for a message object of type '<SaveBinaryFile-response>"
  "3121441b66090c806f9864e12420345a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SaveBinaryFile-response)))
  "Returns md5sum for a message object of type 'SaveBinaryFile-response"
  "3121441b66090c806f9864e12420345a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SaveBinaryFile-response>)))
  "Returns full string definition for message of type '<SaveBinaryFile-response>"
  (cl:format cl:nil "string name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SaveBinaryFile-response)))
  "Returns full string definition for message of type 'SaveBinaryFile-response"
  (cl:format cl:nil "string name~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SaveBinaryFile-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SaveBinaryFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SaveBinaryFile-response
    (cl:cons ':name (name msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SaveBinaryFile)))
  'SaveBinaryFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SaveBinaryFile)))
  'SaveBinaryFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SaveBinaryFile)))
  "Returns string type for a service object of type '<SaveBinaryFile>"
  "file_server/SaveBinaryFile")