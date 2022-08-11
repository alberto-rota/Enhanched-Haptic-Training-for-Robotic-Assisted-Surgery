;; Auto-generated. Do not edit!


(when (boundp 'file_server::SaveBinaryFile)
  (if (not (find-package "FILE_SERVER"))
    (make-package "FILE_SERVER"))
  (shadow 'SaveBinaryFile (find-package "FILE_SERVER")))
(unless (find-package "FILE_SERVER::SAVEBINARYFILE")
  (make-package "FILE_SERVER::SAVEBINARYFILE"))
(unless (find-package "FILE_SERVER::SAVEBINARYFILEREQUEST")
  (make-package "FILE_SERVER::SAVEBINARYFILEREQUEST"))
(unless (find-package "FILE_SERVER::SAVEBINARYFILERESPONSE")
  (make-package "FILE_SERVER::SAVEBINARYFILERESPONSE"))

(in-package "ROS")





(defclass file_server::SaveBinaryFileRequest
  :super ros::object
  :slots (_name _value ))

(defmethod file_server::SaveBinaryFileRequest
  (:init
   (&key
    ((:name __name) "")
    ((:value __value) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _value __value)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; uint8[] _value
    (* 1    (length _value)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; uint8[] _value
     (write-long (length _value) s)
     (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8[] _value
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _value (make-array n :element-type :char))
     (replace _value buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(defclass file_server::SaveBinaryFileResponse
  :super ros::object
  :slots (_name ))

(defmethod file_server::SaveBinaryFileResponse
  (:init
   (&key
    ((:name __name) "")
    )
   (send-super :init)
   (setq _name (string __name))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass file_server::SaveBinaryFile
  :super ros::object
  :slots ())

(setf (get file_server::SaveBinaryFile :md5sum-) "3121441b66090c806f9864e12420345a")
(setf (get file_server::SaveBinaryFile :datatype-) "file_server/SaveBinaryFile")
(setf (get file_server::SaveBinaryFile :request) file_server::SaveBinaryFileRequest)
(setf (get file_server::SaveBinaryFile :response) file_server::SaveBinaryFileResponse)

(defmethod file_server::SaveBinaryFileRequest
  (:response () (instance file_server::SaveBinaryFileResponse :init)))

(setf (get file_server::SaveBinaryFileRequest :md5sum-) "3121441b66090c806f9864e12420345a")
(setf (get file_server::SaveBinaryFileRequest :datatype-) "file_server/SaveBinaryFileRequest")
(setf (get file_server::SaveBinaryFileRequest :definition-)
      "string name
uint8[] value
---
string name

")

(setf (get file_server::SaveBinaryFileResponse :md5sum-) "3121441b66090c806f9864e12420345a")
(setf (get file_server::SaveBinaryFileResponse :datatype-) "file_server/SaveBinaryFileResponse")
(setf (get file_server::SaveBinaryFileResponse :definition-)
      "string name
uint8[] value
---
string name

")



(provide :file_server/SaveBinaryFile "3121441b66090c806f9864e12420345a")


