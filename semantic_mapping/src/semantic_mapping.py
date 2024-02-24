#!/usr/bin/env python

import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler
import numpy as np
import matplotlib.pyplot as plt


def map_callback(occupancy_grid_msg):
    # Assuming the origin of the map is (0, 0, 0) in the real world
    resolution = occupancy_grid_msg.info.resolution
    origin_x = occupancy_grid_msg.info.origin.position.x
    origin_y = occupancy_grid_msg.info.origin.position.y

    # Convert OccupancyGrid data to 2D array
    occupancy_grid = np.array(occupancy_grid_msg.data).reshape((occupancy_grid_msg.info.height, occupancy_grid_msg.info.width))

    # Get indices of occupied cells
    occupied_indices = np.argwhere(occupancy_grid > 0)

    # Use clustering on the 3D points
    # point_cloud_data = []
    # for i, (x, y) in enumerate(occupied_indices):
    #     # Convert x, y to 3D coordinates
    #     point = (x * resolution + origin_x, y * resolution + origin_y, 0.0)  # Fixed z to 0.0
    #     point_cloud_data.append(point)

    #Convert OccupancyGrid data to PointCloud2 data
    point_cloud_data = []
    for y in range(occupancy_grid_msg.info.height):
        for x in range(occupancy_grid_msg.info.width):
            # Occupied grids
            if occupancy_grid_msg.data[y * occupancy_grid_msg.info.width + x] > 0:
                point_cloud_data.append((x * resolution + origin_x, y * resolution + origin_y, 2.0))

    point_cloud_array = np.array(point_cloud_data)

    # Apply K-Means clustering
    if len(point_cloud_data) > 0:
        scaled_data = StandardScaler().fit_transform(point_cloud_array)
        kmeans = KMeans(n_clusters=10, random_state=42)  # Adjust the number of clusters as needed and random_state=42 ensures that the algorithm starts from the same initial centroids everytime
        labels = kmeans.fit_predict(scaled_data)
    
     # Visualize the clustered map
    unique_labels = np.unique(labels)
    colors = plt.cm.get_cmap('tab20', len(unique_labels))

    # Create a dictionary to map labels to colors
    label_color_dict = {label: colors(i)[:3] for i, label in enumerate(unique_labels)}

    for i, label in enumerate(unique_labels):
        cluster_points = point_cloud_array[labels == label]
        plt.scatter(cluster_points[:, 0], cluster_points[:, 1], c=[label_color_dict[label]], label=f'Cluster {label}')

    # Create a PointCloud2 message
    header = occupancy_grid_msg.header
    header.frame_id = "map"
    point_cloud_msg = pc2.create_cloud_xyz32(header, point_cloud_data)

    cluster_names = {0: "Cluster_A", 1: "Cluster_B", 2: "Cluster_C", 3: "Cluster_D", 4: "Cluster_E",5: "Cluster_F", 6: "Cluster_G", 7: "Cluster_H", 8: "Cluster_I", 9: "Cluster_J"}

    # Create MarkerArray for displaying clusters as markers
    marker_array = MarkerArray()
    for i, (x, y, _) in enumerate(point_cloud_array):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CYLINDER
        marker.id = i
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0  
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  
        marker.scale.y = 0.2  
        marker.scale.z = 0.2  
        color = label_color_dict[labels[i]]
        marker.color.r, marker.color.g, marker.color.b = color[0], color[1], color[2]
        marker.color.a = 1.0
        marker.text = cluster_names.get(labels[i], f"Cluster_{labels[i]}") 
        marker.ns = marker.text
        marker_array.markers.append(marker)

    # Create a Text Marker for each point (TEXT_VIEW_FACING)
    text_array = MarkerArray()
    for j, (x, y, _) in enumerate(point_cloud_array):
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.id = labels[j] 
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y-2
        text_marker.pose.position.z = 1.5  
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.5
        text_marker.color.r, text_marker.color.g, text_marker.color.b = 0, 0, 0
        text_marker.color.a = 1.0
        text_marker.text = cluster_names.get(labels[j], f"Cluster_{labels[j]}")
        text_marker.ns = text_marker.text
        text_array.markers.append(text_marker)


    # Publish the Marker message
    markers_publisher.publish(marker_array)
    rospy.loginfo("Semantic Map published!")

    # Publish the Cluster names
    text_publisher.publish(text_array)

    # Publish the PointCloud2 message
    # point_cloud_publisher.publish(point_cloud_msg)
    # rospy.loginfo("Point Cloud published!")

    # Broadcast TF transform from /map to /point_cloud
    tf_broadcaster.sendTransform(
        (origin_x, origin_y, 0.0),  # Translation
        quaternion_from_euler(0, 0, 0),  # Rotation
        rospy.Time.now(),
        "point_cloud",
        "map"
    )

    # plt.title('Clustered Map')
    # plt.xlabel('X-coordinate')
    # plt.ylabel('Y-coordinate')
    # plt.legend()
    # plt.show()



if __name__ == '__main__':
    rospy.init_node('occupancy_grid_clustering')

    # Subscribe to the /map topic
    map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # Publish the PointCloud2 on the /point_cloud topic
    # point_cloud_publisher = rospy.Publisher('/point_cloud', pc2.PointCloud2, queue_size=10000000)

    # Publish the MarkerArray on the /semantic_map topic
    markers_publisher = rospy.Publisher('/semantic_map', MarkerArray, queue_size=10000000)

    # Publish the Text MarkerArray on the /text_markers topic
    text_publisher = rospy.Publisher('/text_markers', MarkerArray, queue_size=10)

    # TF Broadcaster
    tf_broadcaster = tf.TransformBroadcaster()

    rospy.spin()
