; Auto-generated. Do not edit!


(cl:in-package motor_serial-srv)


;//! \htmlinclude motor_serial-request.msg.html

(cl:defclass <motor_serial-request> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0)
   (cmd
    :reader cmd
    :initarg :cmd
    :type cl:fixnum
    :initform 0)
   (data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass motor_serial-request (<motor_serial-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_serial-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_serial-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_serial-srv:<motor_serial-request> is deprecated: use motor_serial-srv:motor_serial-request instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <motor_serial-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_serial-srv:id-val is deprecated.  Use motor_serial-srv:id instead.")
  (id m))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <motor_serial-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_serial-srv:cmd-val is deprecated.  Use motor_serial-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <motor_serial-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_serial-srv:data-val is deprecated.  Use motor_serial-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_serial-request>) ostream)
  "Serializes a message object of type '<motor_serial-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_serial-request>) istream)
  "Deserializes a message object of type '<motor_serial-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cmd)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_serial-request>)))
  "Returns string type for a service object of type '<motor_serial-request>"
  "motor_serial/motor_serialRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_serial-request)))
  "Returns string type for a service object of type 'motor_serial-request"
  "motor_serial/motor_serialRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_serial-request>)))
  "Returns md5sum for a message object of type '<motor_serial-request>"
  "145fd4bef55a3bc00166d50e6ebdc608")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_serial-request)))
  "Returns md5sum for a message object of type 'motor_serial-request"
  "145fd4bef55a3bc00166d50e6ebdc608")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_serial-request>)))
  "Returns full string definition for message of type '<motor_serial-request>"
  (cl:format cl:nil "uint8 id~%uint8 cmd~%int16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_serial-request)))
  "Returns full string definition for message of type 'motor_serial-request"
  (cl:format cl:nil "uint8 id~%uint8 cmd~%int16 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_serial-request>))
  (cl:+ 0
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_serial-request>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_serial-request
    (cl:cons ':id (id msg))
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':data (data msg))
))
;//! \htmlinclude motor_serial-response.msg.html

(cl:defclass <motor_serial-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass motor_serial-response (<motor_serial-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motor_serial-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motor_serial-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_serial-srv:<motor_serial-response> is deprecated: use motor_serial-srv:motor_serial-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <motor_serial-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_serial-srv:data-val is deprecated.  Use motor_serial-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motor_serial-response>) ostream)
  "Serializes a message object of type '<motor_serial-response>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motor_serial-response>) istream)
  "Deserializes a message object of type '<motor_serial-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motor_serial-response>)))
  "Returns string type for a service object of type '<motor_serial-response>"
  "motor_serial/motor_serialResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_serial-response)))
  "Returns string type for a service object of type 'motor_serial-response"
  "motor_serial/motor_serialResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motor_serial-response>)))
  "Returns md5sum for a message object of type '<motor_serial-response>"
  "145fd4bef55a3bc00166d50e6ebdc608")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motor_serial-response)))
  "Returns md5sum for a message object of type 'motor_serial-response"
  "145fd4bef55a3bc00166d50e6ebdc608")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motor_serial-response>)))
  "Returns full string definition for message of type '<motor_serial-response>"
  (cl:format cl:nil "int16 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motor_serial-response)))
  "Returns full string definition for message of type 'motor_serial-response"
  (cl:format cl:nil "int16 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motor_serial-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motor_serial-response>))
  "Converts a ROS message object to a list"
  (cl:list 'motor_serial-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'motor_serial)))
  'motor_serial-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'motor_serial)))
  'motor_serial-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motor_serial)))
  "Returns string type for a service object of type '<motor_serial>"
  "motor_serial/motor_serial")