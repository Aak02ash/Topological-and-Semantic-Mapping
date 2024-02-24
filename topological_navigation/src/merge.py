#!/usr/bin/env python

import rospy
import networkx as nx
from topological_navigation.msg import Node


def merge_maps():

    G = nx.read_graphml('/home/aakash/ohmni_ws/src/topological_mapping/graph/topo1.graphml')

    # Extract nodes
    nodes = G.nodes(data=True)

    # Convert to ROS message format
    ros_nodes = []
    for node_id, data in nodes:
        node = Node()
        node.node = int(node_id)
        node.x_pose = float(data.get('x_pose', 0.0))  
        node.y_pose = float(data.get('y_pose', 0.0))
        node.z_pose = float(data.get('z_pose', 0.0))
        node.yaw = float(data.get('yaw', 0.0))
        ros_nodes.append(node)

    cluster_names = {0: "Cluster_A", 1: "Cluster_B", 2: "Cluster_C", 3: "Cluster_D", 4: "Cluster_E",5: "Cluster_F", 6: "Cluster_G", 7: "Cluster_H", 8: "Cluster_I", 9: "Cluster_J"}

    # Create a mapping from node_id to cluster_name
    node_id_to_cluster_mapping = {}

    # Manually map node IDs to cluster names
    node_id_to_cluster_mapping = {
        10: "Cluster_I",
        11: "Cluster_C",
        3: "Cluster_D",
        5: "Cluster_J",
        6: "Cluster_E",
        7: "Cluster_G",
        8: "Cluster_H",
    }

    print("Node ID to Cluster Mapping:", node_id_to_cluster_mapping)

    # Take user input for the cluster name
    user_input_cluster_name = input("Enter the cluster name: ")

    # Get the node_ids belonging to the user-input cluster name
    nodes_in_cluster = [node_id for node_id, cluster_name in node_id_to_cluster_mapping.items() if cluster_name == user_input_cluster_name]

    # Display node details for the nodes in the specified cluster
    if nodes_in_cluster:
        print(f"Nodes in {user_input_cluster_name}:\n")
        for node_id in nodes_in_cluster:
            node_details = next((node for node in ros_nodes if node.node == node_id), None)
            if node_details:
                print(f"Node ID: {node_details.node}, X: {node_details.x_pose}, Y: {node_details.y_pose}, Z: {node_details.z_pose}, Yaw: {node_details.yaw}")
    else:
        print(f"No nodes found in {user_input_cluster_name}")


if __name__ == '__main__':
    rospy.init_node('merging_maps')

    merge_maps()

    rospy.spin()



    

