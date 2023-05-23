;; Auto-generated. Do not edit!


(when (boundp 'calibration::Cluster)
  (if (not (find-package "CALIBRATION"))
    (make-package "CALIBRATION"))
  (shadow 'Cluster (find-package "CALIBRATION")))
(unless (find-package "CALIBRATION::CLUSTER")
  (make-package "CALIBRATION::CLUSTER"))

(in-package "ROS")
;;//! \htmlinclude Cluster.msg.html


(defclass calibration::Cluster
  :super ros::object
  :slots (_angle_min _angle_increment _x _y _label ))

(defmethod calibration::Cluster
  (:init
   (&key
    ((:angle_min __angle_min) 0.0)
    ((:angle_increment __angle_increment) 0.0)
    ((:x __x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:y __y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:label __label) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _angle_min (float __angle_min))
   (setq _angle_increment (float __angle_increment))
   (setq _x __x)
   (setq _y __y)
   (setq _label __label)
   self)
  (:angle_min
   (&optional __angle_min)
   (if __angle_min (setq _angle_min __angle_min)) _angle_min)
  (:angle_increment
   (&optional __angle_increment)
   (if __angle_increment (setq _angle_increment __angle_increment)) _angle_increment)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:serialization-length
   ()
   (+
    ;; float32 _angle_min
    4
    ;; float32 _angle_increment
    4
    ;; float32[] _x
    (* 4    (length _x)) 4
    ;; float32[] _y
    (* 4    (length _y)) 4
    ;; int32[] _label
    (* 4    (length _label)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle_min
       (sys::poke _angle_min (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_increment
       (sys::poke _angle_increment (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32[] _x
     (write-long (length _x) s)
     (dotimes (i (length _x))
       (sys::poke (elt _x i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[] _y
     (write-long (length _y) s)
     (dotimes (i (length _y))
       (sys::poke (elt _y i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int32[] _label
     (write-long (length _label) s)
     (dotimes (i (length _label))
       (write-long (elt _label i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle_min
     (setq _angle_min (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_increment
     (setq _angle_increment (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32[] _x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _x i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32[] _y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _y i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; int32[] _label
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _label (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _label i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get calibration::Cluster :md5sum-) "4b2b1ab2620a329ffa16902476b2bdef")
(setf (get calibration::Cluster :datatype-) "calibration/Cluster")
(setf (get calibration::Cluster :definition-)
      "float32 angle_min
float32 angle_increment
float32[] x
float32[] y
int32[] label
")



(provide :calibration/Cluster "4b2b1ab2620a329ffa16902476b2bdef")


