string uuid                         # A unique label for that object or a universal ID
geometry_msgs/Pose pose             # The position and orientation of each object
float64 roundness                   # The circulatiry of the object
float64 bwratio                     # The ratio between the inside and outside circle
float64 esterror                    # The estimate error of the tracking
geometry_msgs/Vector3 objectsize    # dimentions of the objects being tracked x: width, y: length, z: height