; Auto-generated. Do not edit!


(cl:in-package rover_control-msg)


;//! \htmlinclude corner.msg.html

(cl:defclass <corner> (roslisp-msg-protocol:ros-message)
  ((wheel_pos_actual
    :reader wheel_pos_actual
    :initarg :wheel_pos_actual
    :type cl:float
    :initform 0.0)
   (wheel_pos_desired
    :reader wheel_pos_desired
    :initarg :wheel_pos_desired
    :type cl:float
    :initform 0.0)
   (wheel_vel_actual
    :reader wheel_vel_actual
    :initarg :wheel_vel_actual
    :type cl:float
    :initform 0.0)
   (wheel_vel_desired
    :reader wheel_vel_desired
    :initarg :wheel_vel_desired
    :type cl:float
    :initform 0.0)
   (steer_pos_actual
    :reader steer_pos_actual
    :initarg :steer_pos_actual
    :type cl:float
    :initform 0.0)
   (steer_pos_desired
    :reader steer_pos_desired
    :initarg :steer_pos_desired
    :type cl:float
    :initform 0.0)
   (steer_vel_actual
    :reader steer_vel_actual
    :initarg :steer_vel_actual
    :type cl:float
    :initform 0.0)
   (steer_vel_desired
    :reader steer_vel_desired
    :initarg :steer_vel_desired
    :type cl:float
    :initform 0.0))
)

(cl:defclass corner (<corner>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <corner>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'corner)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_control-msg:<corner> is deprecated: use rover_control-msg:corner instead.")))

(cl:ensure-generic-function 'wheel_pos_actual-val :lambda-list '(m))
(cl:defmethod wheel_pos_actual-val ((m <corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:wheel_pos_actual-val is deprecated.  Use rover_control-msg:wheel_pos_actual instead.")
  (wheel_pos_actual m))

(cl:ensure-generic-function 'wheel_pos_desired-val :lambda-list '(m))
(cl:defmethod wheel_pos_desired-val ((m <corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:wheel_pos_desired-val is deprecated.  Use rover_control-msg:wheel_pos_desired instead.")
  (wheel_pos_desired m))

(cl:ensure-generic-function 'wheel_vel_actual-val :lambda-list '(m))
(cl:defmethod wheel_vel_actual-val ((m <corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:wheel_vel_actual-val is deprecated.  Use rover_control-msg:wheel_vel_actual instead.")
  (wheel_vel_actual m))

(cl:ensure-generic-function 'wheel_vel_desired-val :lambda-list '(m))
(cl:defmethod wheel_vel_desired-val ((m <corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:wheel_vel_desired-val is deprecated.  Use rover_control-msg:wheel_vel_desired instead.")
  (wheel_vel_desired m))

(cl:ensure-generic-function 'steer_pos_actual-val :lambda-list '(m))
(cl:defmethod steer_pos_actual-val ((m <corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:steer_pos_actual-val is deprecated.  Use rover_control-msg:steer_pos_actual instead.")
  (steer_pos_actual m))

(cl:ensure-generic-function 'steer_pos_desired-val :lambda-list '(m))
(cl:defmethod steer_pos_desired-val ((m <corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:steer_pos_desired-val is deprecated.  Use rover_control-msg:steer_pos_desired instead.")
  (steer_pos_desired m))

(cl:ensure-generic-function 'steer_vel_actual-val :lambda-list '(m))
(cl:defmethod steer_vel_actual-val ((m <corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:steer_vel_actual-val is deprecated.  Use rover_control-msg:steer_vel_actual instead.")
  (steer_vel_actual m))

(cl:ensure-generic-function 'steer_vel_desired-val :lambda-list '(m))
(cl:defmethod steer_vel_desired-val ((m <corner>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:steer_vel_desired-val is deprecated.  Use rover_control-msg:steer_vel_desired instead.")
  (steer_vel_desired m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <corner>) ostream)
  "Serializes a message object of type '<corner>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wheel_pos_actual))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wheel_pos_desired))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wheel_vel_actual))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wheel_vel_desired))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steer_pos_actual))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steer_pos_desired))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steer_vel_actual))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'steer_vel_desired))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <corner>) istream)
  "Deserializes a message object of type '<corner>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wheel_pos_actual) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wheel_pos_desired) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wheel_vel_actual) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wheel_vel_desired) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_pos_actual) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_pos_desired) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_vel_actual) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'steer_vel_desired) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<corner>)))
  "Returns string type for a message object of type '<corner>"
  "rover_control/corner")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'corner)))
  "Returns string type for a message object of type 'corner"
  "rover_control/corner")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<corner>)))
  "Returns md5sum for a message object of type '<corner>"
  "f28300dc96da5592dc5c99f06bac2854")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'corner)))
  "Returns md5sum for a message object of type 'corner"
  "f28300dc96da5592dc5c99f06bac2854")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<corner>)))
  "Returns full string definition for message of type '<corner>"
  (cl:format cl:nil "float64 wheel_pos_actual~%float64 wheel_pos_desired~%float64 wheel_vel_actual~%float64 wheel_vel_desired~%float64 steer_pos_actual~%float64 steer_pos_desired~%float64 steer_vel_actual~%float64 steer_vel_desired~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'corner)))
  "Returns full string definition for message of type 'corner"
  (cl:format cl:nil "float64 wheel_pos_actual~%float64 wheel_pos_desired~%float64 wheel_vel_actual~%float64 wheel_vel_desired~%float64 steer_pos_actual~%float64 steer_pos_desired~%float64 steer_vel_actual~%float64 steer_vel_desired~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <corner>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <corner>))
  "Converts a ROS message object to a list"
  (cl:list 'corner
    (cl:cons ':wheel_pos_actual (wheel_pos_actual msg))
    (cl:cons ':wheel_pos_desired (wheel_pos_desired msg))
    (cl:cons ':wheel_vel_actual (wheel_vel_actual msg))
    (cl:cons ':wheel_vel_desired (wheel_vel_desired msg))
    (cl:cons ':steer_pos_actual (steer_pos_actual msg))
    (cl:cons ':steer_pos_desired (steer_pos_desired msg))
    (cl:cons ':steer_vel_actual (steer_vel_actual msg))
    (cl:cons ':steer_vel_desired (steer_vel_desired msg))
))
