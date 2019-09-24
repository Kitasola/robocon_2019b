; Auto-generated. Do not edit!


(cl:in-package three_omuni-msg)


;//! \htmlinclude button.msg.html

(cl:defclass <button> (roslisp-msg-protocol:ros-message)
  ((move_angle
    :reader move_angle
    :initarg :move_angle
    :type cl:float
    :initform 0.0)
   (move_speed
    :reader move_speed
    :initarg :move_speed
    :type cl:float
    :initform 0.0)
   (turn_right
    :reader turn_right
    :initarg :turn_right
    :type cl:boolean
    :initform cl:nil)
   (turn_left
    :reader turn_left
    :initarg :turn_left
    :type cl:boolean
    :initform cl:nil)
   (arm_data
    :reader arm_data
    :initarg :arm_data
    :type cl:boolean
    :initform cl:nil)
   (calibration
    :reader calibration
    :initarg :calibration
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass button (<button>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <button>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'button)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name three_omuni-msg:<button> is deprecated: use three_omuni-msg:button instead.")))

(cl:ensure-generic-function 'move_angle-val :lambda-list '(m))
(cl:defmethod move_angle-val ((m <button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader three_omuni-msg:move_angle-val is deprecated.  Use three_omuni-msg:move_angle instead.")
  (move_angle m))

(cl:ensure-generic-function 'move_speed-val :lambda-list '(m))
(cl:defmethod move_speed-val ((m <button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader three_omuni-msg:move_speed-val is deprecated.  Use three_omuni-msg:move_speed instead.")
  (move_speed m))

(cl:ensure-generic-function 'turn_right-val :lambda-list '(m))
(cl:defmethod turn_right-val ((m <button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader three_omuni-msg:turn_right-val is deprecated.  Use three_omuni-msg:turn_right instead.")
  (turn_right m))

(cl:ensure-generic-function 'turn_left-val :lambda-list '(m))
(cl:defmethod turn_left-val ((m <button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader three_omuni-msg:turn_left-val is deprecated.  Use three_omuni-msg:turn_left instead.")
  (turn_left m))

(cl:ensure-generic-function 'arm_data-val :lambda-list '(m))
(cl:defmethod arm_data-val ((m <button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader three_omuni-msg:arm_data-val is deprecated.  Use three_omuni-msg:arm_data instead.")
  (arm_data m))

(cl:ensure-generic-function 'calibration-val :lambda-list '(m))
(cl:defmethod calibration-val ((m <button>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader three_omuni-msg:calibration-val is deprecated.  Use three_omuni-msg:calibration instead.")
  (calibration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <button>) ostream)
  "Serializes a message object of type '<button>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'move_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'move_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'turn_right) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'turn_left) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'arm_data) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'calibration) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <button>) istream)
  "Deserializes a message object of type '<button>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'move_angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'move_speed) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'turn_right) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'turn_left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'arm_data) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'calibration) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<button>)))
  "Returns string type for a message object of type '<button>"
  "three_omuni/button")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'button)))
  "Returns string type for a message object of type 'button"
  "three_omuni/button")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<button>)))
  "Returns md5sum for a message object of type '<button>"
  "c703a9d0e274f7444bd1093549d0f0d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'button)))
  "Returns md5sum for a message object of type 'button"
  "c703a9d0e274f7444bd1093549d0f0d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<button>)))
  "Returns full string definition for message of type '<button>"
  (cl:format cl:nil "float64 move_angle~%float64 move_speed~%bool turn_right~%bool turn_left~%bool arm_data~%bool calibration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'button)))
  "Returns full string definition for message of type 'button"
  (cl:format cl:nil "float64 move_angle~%float64 move_speed~%bool turn_right~%bool turn_left~%bool arm_data~%bool calibration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <button>))
  (cl:+ 0
     8
     8
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <button>))
  "Converts a ROS message object to a list"
  (cl:list 'button
    (cl:cons ':move_angle (move_angle msg))
    (cl:cons ':move_speed (move_speed msg))
    (cl:cons ':turn_right (turn_right msg))
    (cl:cons ':turn_left (turn_left msg))
    (cl:cons ':arm_data (arm_data msg))
    (cl:cons ':calibration (calibration msg))
))
