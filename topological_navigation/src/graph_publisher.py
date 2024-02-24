#!/usr/bin/python


import networkx as nx
import rospy
from topological_navigation.msg import Node, Edge

def publish_nodes_and_edges():
    # Read .graphml file
    G = nx.read_graphml('/home/aakash/ohmni_ws/src/topological_mapping/graph/topo1.graphml')

    # Extract nodes and edges
    nodes = G.nodes(data=True)
    edges = G.edges(data=True)

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

    ros_edges = [Edge(source=edge[0], target=edge[1]) for edge in edges]

    # List of nodes and edges
    nodes = {int(node.node): (node.x_pose, node.y_pose, node.z_pose, node.yaw) for node in ros_nodes}
    edges = [(int(edge.source), int(edge.target)) for edge in ros_edges]

    print("Nodes:", nodes)
    print("Edges:", edges)

    # Initialize ROS node
    rospy.init_node('graph_publisher')

    # Publish nodes and edges as ROS topics
    node_pub = rospy.Publisher('/nodes_topic', Node, queue_size=10)
    edge_pub = rospy.Publisher('/edges_topic', Edge, queue_size=10)

    # Publish nodes
    for node in ros_nodes:
        node_pub.publish(node)
        rospy.sleep(1.0)  # Adjust sleep duration as needed

    # Publish edges
    for edge in ros_edges:
        edge_pub.publish(edge)
        rospy.sleep(1.0)  # Adjust sleep duration as needed

    rospy.loginfo("Nodes and edges published!")
    rospy.spin()

if __name__ == "__main__":
        publish_nodes_and_edges()


