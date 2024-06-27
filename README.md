**ReadData Node**
The ReadData node is designed to process LiDAR scan data to identify and classify moving and stationary points. It subscribes to the /scan topic to receive LaserScan messages and publishes the processed data to three topics: /moving_points, /stationary_points, and /centroids.

**Initialization and Subscription**
The node initializes by setting up subscriptions and publishers. It subscribes to the /scan topic to receive LiDAR scan data and sets up publishers to share moving points, stationary points, and centroids.

**Data Processing**
In the posecallback method, the node processes incoming LiDAR data. If it's the first scan, it stores it as a reference. For subsequent scans, it differentiates between moving and stationary points by comparing the distances from the current scan to the first scan. Points exceeding a defined threshold are classified as moving, while others are classified as stationary.

**Clustering**
The DBscan method applies the DBSCAN clustering algorithm to the moving points to identify clusters, which represent individuals. The algorithm groups points based on proximity, defined by the eps (maximum distance between points to be considered part of the same cluster) and min_samples (minimum number of points required to form a cluster) parameters. Identified clusters' centroids are then published to the /centroids topic.

**Publishing Data**
The publish_cloud method publishes the processed point clouds to the respective topics. Each point cloud is tagged with a header indicating the frame of reference.

<p align="center">
  <img src="https://github.com/Nat172001/Laser-Tracking/blob/main/Bag2.gif" width="400" />
  <img src="https://github.com/Nat172001/Laser-Tracking/blob/main/Bag7.gif" width="400" />
</p>

**TrackData Node**
The TrackData node tracks individuals over time using the centroids published by the ReadData node. It subscribes to the /centroids topic and publishes tracked objects as markers to the /tracked_objects topic.

**Initialization and Subscription**
The node initializes by setting up a subscription to the /centroids topic to receive centroid data and a publisher to share tracked objects as markers. It also initializes parameters for tracking objects, including a distance threshold for associating new centroids with existing tracked objects.

**Centroid Tracking**
In the receive_centroids_callback method, the node updates the list of tracked objects based on the received centroids. It uses Euclidean distance to match new centroids with existing tracked objects. If a centroid is within a defined distance of an existing tracked object, it updates the object's position and velocity. If not, it starts tracking a new object.

**Velocity Calculation**
The TrackedObject class includes methods for updating the time and velocity of each tracked object. Velocity is calculated based on the change in position over time, allowing the node to estimate future positions and improve tracking accuracy.

**Publishing Tracked Objects**
The publish_tracked_objects method publishes the tracked objects as markers. It creates visual markers for each tracked object and a line strip marker to show the object's movement history. Markers are published to the /tracked_objects topic, providing a visual representation of tracked individuals' positions and movements.


