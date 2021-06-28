# syracuse


We will be using PCL (Point Cloud Library) to deal with our point clouds and perform actions on them as well as to visualize the point clouds themselves.


pcHandler class will handle the point cloud processing and includes functions such as: LoadPoints, Cluster, Slice and View according to all the different requirements.

Clustering is done using eucledian clustering and searching through a kd-tree which ensures a search time of O(log(n)) for Three dimensional points (XYZ).

Slicing is done by projecting onto a plane the constant value for the given axis provided and returning that plane.



stateHandler will handle the state aspects and monitor the velocity it includes functions such as: LoadState, DetectMotion, DetectVelocity.
If current speed surpasses the velocity limit an exception will be raised that is handled by main.cpp and this will cause us to ignore the relevant point clouds as well as to log to file. In addition, any changes to state will be logged to file syracuse.log
