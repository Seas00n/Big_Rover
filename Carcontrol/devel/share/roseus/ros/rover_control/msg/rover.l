;; Auto-generated. Do not edit!


(when (boundp 'rover_control::rover)
  (if (not (find-package "ROVER_CONTROL"))
    (make-package "ROVER_CONTROL"))
  (shadow 'rover (find-package "ROVER_CONTROL")))
(unless (find-package "ROVER_CONTROL::ROVER")
  (make-package "ROVER_CONTROL::ROVER"))

(in-package "ROS")
;;//! \htmlinclude rover.msg.html


(defclass rover_control::rover
  :super ros::object
  :slots (_rover_px _rover_py _rover_vx _rover_vy _rover_w _rover_state ))

(defmethod rover_control::rover
  (:init
   (&key
    ((:rover_px __rover_px) 0.0)
    ((:rover_py __rover_py) 0.0)
    ((:rover_vx __rover_vx) 0.0)
    ((:rover_vy __rover_vy) 0.0)
    ((:rover_w __rover_w) 0.0)
    ((:rover_state __rover_state) 0)
    )
   (send-super :init)
   (setq _rover_px (float __rover_px))
   (setq _rover_py (float __rover_py))
   (setq _rover_vx (float __rover_vx))
   (setq _rover_vy (float __rover_vy))
   (setq _rover_w (float __rover_w))
   (setq _rover_state (round __rover_state))
   self)
  (:rover_px
   (&optional __rover_px)
   (if __rover_px (setq _rover_px __rover_px)) _rover_px)
  (:rover_py
   (&optional __rover_py)
   (if __rover_py (setq _rover_py __rover_py)) _rover_py)
  (:rover_vx
   (&optional __rover_vx)
   (if __rover_vx (setq _rover_vx __rover_vx)) _rover_vx)
  (:rover_vy
   (&optional __rover_vy)
   (if __rover_vy (setq _rover_vy __rover_vy)) _rover_vy)
  (:rover_w
   (&optional __rover_w)
   (if __rover_w (setq _rover_w __rover_w)) _rover_w)
  (:rover_state
   (&optional __rover_state)
   (if __rover_state (setq _rover_state __rover_state)) _rover_state)
  (:serialization-length
   ()
   (+
    ;; float64 _rover_px
    8
    ;; float64 _rover_py
    8
    ;; float64 _rover_vx
    8
    ;; float64 _rover_vy
    8
    ;; float64 _rover_w
    8
    ;; int32 _rover_state
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _rover_px
       (sys::poke _rover_px (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rover_py
       (sys::poke _rover_py (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rover_vx
       (sys::poke _rover_vx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rover_vy
       (sys::poke _rover_vy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rover_w
       (sys::poke _rover_w (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int32 _rover_state
       (write-long _rover_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _rover_px
     (setq _rover_px (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rover_py
     (setq _rover_py (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rover_vx
     (setq _rover_vx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rover_vy
     (setq _rover_vy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rover_w
     (setq _rover_w (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int32 _rover_state
     (setq _rover_state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get rover_control::rover :md5sum-) "8939377560b48465bde83d9b3af7dfb7")
(setf (get rover_control::rover :datatype-) "rover_control/rover")
(setf (get rover_control::rover :definition-)
      "float64 rover_px
float64 rover_py
float64 rover_vx
float64 rover_vy
float64 rover_w
int32 rover_state
")



(provide :rover_control/rover "8939377560b48465bde83d9b3af7dfb7")


