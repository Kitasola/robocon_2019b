;; Auto-generated. Do not edit!


(when (boundp 'three_omuni::button)
  (if (not (find-package "THREE_OMUNI"))
    (make-package "THREE_OMUNI"))
  (shadow 'button (find-package "THREE_OMUNI")))
(unless (find-package "THREE_OMUNI::BUTTON")
  (make-package "THREE_OMUNI::BUTTON"))

(in-package "ROS")
;;//! \htmlinclude button.msg.html


(defclass three_omuni::button
  :super ros::object
  :slots (_move_angle _move_speed _turn_right _turn_left _arm_data _calibration ))

(defmethod three_omuni::button
  (:init
   (&key
    ((:move_angle __move_angle) 0.0)
    ((:move_speed __move_speed) 0.0)
    ((:turn_right __turn_right) nil)
    ((:turn_left __turn_left) nil)
    ((:arm_data __arm_data) nil)
    ((:calibration __calibration) nil)
    )
   (send-super :init)
   (setq _move_angle (float __move_angle))
   (setq _move_speed (float __move_speed))
   (setq _turn_right __turn_right)
   (setq _turn_left __turn_left)
   (setq _arm_data __arm_data)
   (setq _calibration __calibration)
   self)
  (:move_angle
   (&optional __move_angle)
   (if __move_angle (setq _move_angle __move_angle)) _move_angle)
  (:move_speed
   (&optional __move_speed)
   (if __move_speed (setq _move_speed __move_speed)) _move_speed)
  (:turn_right
   (&optional __turn_right)
   (if __turn_right (setq _turn_right __turn_right)) _turn_right)
  (:turn_left
   (&optional __turn_left)
   (if __turn_left (setq _turn_left __turn_left)) _turn_left)
  (:arm_data
   (&optional __arm_data)
   (if __arm_data (setq _arm_data __arm_data)) _arm_data)
  (:calibration
   (&optional __calibration)
   (if __calibration (setq _calibration __calibration)) _calibration)
  (:serialization-length
   ()
   (+
    ;; float64 _move_angle
    8
    ;; float64 _move_speed
    8
    ;; bool _turn_right
    1
    ;; bool _turn_left
    1
    ;; bool _arm_data
    1
    ;; bool _calibration
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _move_angle
       (sys::poke _move_angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _move_speed
       (sys::poke _move_speed (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _turn_right
       (if _turn_right (write-byte -1 s) (write-byte 0 s))
     ;; bool _turn_left
       (if _turn_left (write-byte -1 s) (write-byte 0 s))
     ;; bool _arm_data
       (if _arm_data (write-byte -1 s) (write-byte 0 s))
     ;; bool _calibration
       (if _calibration (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _move_angle
     (setq _move_angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _move_speed
     (setq _move_speed (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _turn_right
     (setq _turn_right (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _turn_left
     (setq _turn_left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _arm_data
     (setq _arm_data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _calibration
     (setq _calibration (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get three_omuni::button :md5sum-) "c703a9d0e274f7444bd1093549d0f0d5")
(setf (get three_omuni::button :datatype-) "three_omuni/button")
(setf (get three_omuni::button :definition-)
      "float64 move_angle
float64 move_speed
bool turn_right
bool turn_left
bool arm_data
bool calibration


")



(provide :three_omuni/button "c703a9d0e274f7444bd1093549d0f0d5")


