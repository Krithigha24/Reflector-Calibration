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
  :slots (_index _label ))

(defmethod calibration::Cluster
  (:init
   (&key
    ((:index __index) (make-array 0 :initial-element 0 :element-type :integer))
    ((:label __label) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _index __index)
   (setq _label __label)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:serialization-length
   ()
   (+
    ;; int32[] _index
    (* 4    (length _index)) 4
    ;; int32[] _label
    (* 4    (length _label)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _index
     (write-long (length _index) s)
     (dotimes (i (length _index))
       (write-long (elt _index i) s)
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
   ;; int32[] _index
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _index (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _index i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
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

(setf (get calibration::Cluster :md5sum-) "9b0088670d95be7298b39001111310db")
(setf (get calibration::Cluster :datatype-) "calibration/Cluster")
(setf (get calibration::Cluster :definition-)
      "#float32 angle_min
#float32 angle_increment
int32[] index
int32[] label

")



(provide :calibration/Cluster "9b0088670d95be7298b39001111310db")


