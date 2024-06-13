import rclpy
import numpy as np
from math import sin, cos
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from sklearn.cluster import DBSCAN

class ReadData(Node):
    def __init__(self):
        super().__init__("scan")
        self.create_subscription(LaserScan,"/scan",self.posecallback,100)
        self.moving_points_publisher_ = self.create_publisher(PointCloud, "/moving_points", 10)
        self.stationary_points_publisher_ = self.create_publisher(PointCloud, "/stationary_points", 10)
        self.centroids_publisher_ = self.create_publisher(PointCloud, "/centroids", 10)
        self.first_scan_data = None
        self.previous_scan = None
        self.current_scan = None
        self.previous_points = []
        self.current_points = []
        self.threshold=0.5

    def posecallback(self, msg:LaserScan):
        self.previous_scan = self.current_scan
        self.current_scan = msg

        if self.first_scan_data is None:
            self.first_scan_data = self.current_scan
        else:
            move, stat = self.process_scan()
            mgroups, centroid = self.DBscan(move)
            self.publish_cloud(move, "moving_points")
            self.publish_cloud(stat, "stationary_points")
            if centroid is not None:
                self.publish_cloud(centroid, "centroids")

    def process_scan(self):
        moving_points = []
        stationary_points = []
        for i in range(len(self.current_scan.ranges)):
            if self.current_scan.ranges[i] < self.first_scan_data.ranges[i] - self.threshold:
                moving_points.append(Point32(
                    x=self.current_scan.ranges[i] * cos(self.current_scan.angle_min+i*self.current_scan.angle_increment),
                    y=self.current_scan.ranges[i] * sin(self.current_scan.angle_min+i*self.current_scan.angle_increment),
                    z=0.0))
            else:
                stationary_points.append(Point32(
                    x=self.current_scan.ranges[i] * cos(self.current_scan.angle_min+i*self.current_scan.angle_increment),
                    y=self.current_scan.ranges[i] * sin(self.current_scan.angle_min+i*self.current_scan.angle_increment),
                    z=0.0))

        return PointCloud(header=self.create_header("world"), points=moving_points), \
               PointCloud(header=self.create_header("world"), points=stationary_points)

    def publish_cloud(self, cloud, topic):
        cloud.header.frame_id = "world"
        if topic == "moving_points":
            self.moving_points_publisher_.publish(cloud)
        elif topic == "stationary_points":
            self.stationary_points_publisher_.publish(cloud)
        elif topic == "centroids":
            self.centroids_publisher_.publish(cloud)

    def create_header(self, frame_id):
        header = PointCloud().header
        header.frame_id = frame_id
        return header

    def DBscan(self, moving_points):
        points = np.array([[p.x, p.y] for p in moving_points.points])

        if len(points) < 2:
            print('Not enough points for clustering.')
            return [], None

        db_eps = 0.35
        db_min_samples = 8

        clustering = DBSCAN(eps=db_eps, min_samples=db_min_samples).fit(points)
        labels = clustering.labels_
        centroids = []
        groups = []
        for cluster_id in set(labels):
            if cluster_id == -1:
                continue

            cluster_points = points[labels == cluster_id]
            groups.append(cluster_points)

            centroid = np.mean(cluster_points, axis=0)
            centroids.append(Point32(x=centroid[0], y=centroid[1], z=0.0))

        centroid_cloud = PointCloud(header=moving_points.header, points=centroids)
        return groups, centroid_cloud

def main(args=None):
    rclpy.init(args=args)
    my_node = ReadData()
    rclpy.spin(my_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
