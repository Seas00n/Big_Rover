; Auto-generated. Do not edit!


(cl:in-package rover_control-msg)


;//! \htmlinclude rover.msg.html

(cl:defclass <rover> (roslisp-msg-protocol:ros-message)
  ((rover_px
    :reader rover_px
    :initarg :rover_px
    :type cl:float
    :initform 0.0)
   (rover_py
    :reader rover_py
    :initarg :rover_py
    :type cl:float
    :initform 0.0)
   (rover_vx
    :reader rover_vx
    :initarg :rover_vx
    :type cl:float
    :initform 0.0)
   (rover_vy
    :reader rover_vy
    :initarg :rover_vy
    :type cl:float
    :initform 0.0)
   (rover_w
    :reader rover_w
    :initarg :rover_w
    :type cl:float
    :initform 0.0)
   (rover_state
    :reader rover_state
    :initarg :rover_state
    :type cl:integer
    :initform 0))
)

(cl:defclass rover (<rover>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rover>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rover)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rover_control-msg:<rover> is deprecated: use rover_control-msg:rover instead.")))

(cl:ensure-generic-function 'rover_px-val :lambda-list '(m))
(cl:defmethod rover_px-val ((m <rover>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:rover_px-val is deprecated.  Use rover_control-msg:rover_px instead.")
  (rover_px m))

(cl:ensure-generic-function 'rover_py-val :lambda-list '(m))
(cl:defmethod rover_py-val ((m <rover>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:rover_py-val is deprecated.  Use rover_control-msg:rover_py instead.")
  (rover_py m))

(cl:ensure-generic-function 'rover_vx-val :lambda-list '(m))
(cl:defmethod rover_vx-val ((m <rover>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:rover_vx-val is deprecated.  Use rover_control-msg:rover_vx instead.")
  (rover_vx m))

(cl:ensure-generic-function 'rover_vy-val :lambda-list '(m))
(cl:defmethod rover_vy-val ((m <rover>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:rover_vy-val is deprecated.  Use rover_control-msg:rover_vy instead.")
  (rover_vy m))

(cl:ensure-generic-function 'rover_w-val :lambda-list '(m))
(cl:defmethod rover_w-val ((m <rover>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:rover_w-val is deprecated.  Use rover_control-msg:rover_w instead.")
  (rover_w m))

(cl:ensure-generic-function 'rover_state-val :lambda-list '(m))
(cl:defmethod rover_state-val ((m <rover>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rover_control-msg:rover_state-val is deprecated.  Use rover_control-msg:rover_state instead.")
  (rover_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rover>) ostream)
  "Serializes a message object of type '<rover>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rover_px))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rover_py))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rover_vx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rover_vy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rover_w))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'rover_state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rover>) istream)
  "Deserializes a message object of type '<rover>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rover_px) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rover_py) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rover_vx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rover_vy) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rover_w) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rover_state) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rover>)))
  "Returns string type for a message object of type '<rover>"
  "rover_control/rover")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rover)))
  "Returns string type for a message object of type 'rover"
  "rover_control/rover")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rover>)))
  "Returns md5sum for a message object of type '<rover>"
  "8939377560b48465bde83d9b3af7dfb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rover)))
  "Returns md5sum for a message object of type 'rover"
  "8939377560b48465bde83d9b3af7dfb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rover>)))
  "Returns full string definition for message of type '<rover>"
  (cl:format cl:nil "float64 rover_px~%float64 rover_py~%float64 rover_vx~%float64 rover_vy~%float64 rover_w~%int32 rover_state~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rover)))
  "Returns full string definition for message of type 'rover"
  (cl:format cl:nil "float64 rover_px~%float64 rover_py~%float64 rover_vx~%float64 rover_vy~%float64 rover_w~%int32 rover_state~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rover>))
  (cl:+ 0
     8
     8
     8
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rover>))
  "Converts a ROS message object to a list"
  (cl:list 'rover
    (cl:cons ':rover_px (rover_px msg))
    (cl:cons ':rover_py (rover_py msg))
    (cl:cons ':rover_vx (rover_vx msg))
    (cl:cons ':rover_vy (rover_vy msg))
    (cl:cons ':rover_w (rover_w msg))
    (cl:cons ':rover_state (rover_state msg))
))
