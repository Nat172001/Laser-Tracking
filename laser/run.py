import rclpy
import random
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point32, Point
from sensor_msgs.msg import PointCloud
from visualization_msgs.msg import Marker, MarkerArray
import time  

class TrackedObject:
    def __init__(self, object_id, centroid):
        self.id = object_id
        self.centroid = centroid
        self.history = [centroid]
        self.updates = 1
        self.color = self.generate_random_number()
        self.time_updated = None
        self.velocity = np.array([0, 0]) 

    def generate_random_number(self):
        return [random.randint(0, 255)/255, random.randint(0, 255)/255, random.randint(0, 255)/255]

    def update_time(self):
        self.time_updated = round(time.time(), 6)

    def update_velocity(self, new_centroid):
        delta_time = time.time() - self.time_updated
        if delta_time > 0:
            self.velocity = np.array([(new_centroid.x - self.centroid.x) / delta_time,
                                      (new_centroid.y - self.centroid.y) / delta_time])
        else:
            self.velocity = np.array([0, 0])


class TrackData(Node):
    def __init__(self):
        super().__init__("track_data")
        self.subscription = self.create_subscription(PointCloud, "/centroids", self.receive_centroids_callback, 1000)
        self.publisher = self.create_publisher(MarkerArray,"/tracked_objects", 1000)
        self.header = None
        self.tracked_objects = []  
        self.id = 0  
        self.dis_threshold = 0.42 #don't change more tha 0.8- bag 2
        self.time_threshold = 1.5

    def receive_centroids_callback(self, msg:PointCloud):
        centroids = msg.points
        self.header = msg.header
        self.update_tracked_objects(centroids)
        self.publish_tracked_objects()

    def update_tracked_objects(self, centroids):
        current_time = time.time()
        used_centroids = []  
        for index, centroid in enumerate(centroids):
            min_distance = float('inf')
            closest_object = None
            closest_index = None  
            for tracked_object in self.tracked_objects:
                delta_time = current_time - tracked_object.time_updated
                if delta_time > 0:
                    estimated_x = tracked_object.centroid.x + tracked_object.velocity[0] *delta_time
                    estimated_y = tracked_object.centroid.y + tracked_object.velocity[1] *delta_time
                    distance = self.ed(Point32(x=estimated_x, y=estimated_y), centroid)

                    if distance < self.dis_threshold and distance < min_distance and index not in used_centroids:
                        min_distance = distance
                        closest_object = tracked_object
                        closest_index = index

            if closest_object:
                closest_object.update_velocity(centroid)
                closest_object.centroid = centroid
                closest_object.history.append(centroid)  
                closest_object.updates += 1  
                closest_object.update_time()  
                if closest_index is not None:
                    used_centroids.append(closest_index)  
        for i, centroid in enumerate(centroids):
            if i not in used_centroids:
                self.start_tracking_new_object(centroid)


    def ed(self, tracked_centroid, new_centroid):
        distance = np.sqrt((tracked_centroid.x - new_centroid.x) ** 2 + (tracked_centroid.y - new_centroid.y) ** 2)
        return distance 

    def start_tracking_new_object(self, centroid):
        self.id += 1
        new_id = self.id
        new_object = TrackedObject(new_id, centroid)
        new_object.update_time()  
        self.tracked_objects.append(new_object)
    
    def publish_tracked_objects(self):
        marker_array = MarkerArray()  

        for tracked_object in self.tracked_objects:
            if tracked_object.updates > 3 and self.ed(tracked_object.centroid, tracked_object.history[0]) > 0.5:
                marker = Marker()
                marker.header = self.header
                # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
                marker.type = 2
                marker.id = tracked_object.id

                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3

                marker.color.r = tracked_object.color[0]
                marker.color.g = tracked_object.color[1]
                marker.color.b = tracked_object.color[2]
                marker.color.a = 1.0

                marker.pose.position.x = tracked_object.centroid.x
                marker.pose.position.y = tracked_object.centroid.y
                marker.pose.position.z = tracked_object.centroid.z

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                marker_array.markers.append(marker)
                
                line_strip = Marker()
                line_strip.header = self.header
                line_strip.type = Marker.LINE_STRIP
                line_strip.id = tracked_object.id + 1000 #to avoid conflict
                line_strip.scale.x = 0.1  
                line_strip.color.r = tracked_object.color[0]
                line_strip.color.g = tracked_object.color[1]
                line_strip.color.b = tracked_object.color[2]
                line_strip.color.a = 1.0
                
                line_strip.points = [Point(x=p.x, y=p.y, z=p.z) for p in tracked_object.history]
                
                marker_array.markers.append(line_strip)
                
        self.publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    track_data_node = TrackData()
    rclpy.spin(track_data_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
