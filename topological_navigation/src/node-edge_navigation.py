#!/usr/bin/env python

import rospy
import networkx as nx
from topological_navigation.msg import Node, Edge
from actionlib import SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

class NodeEdgeNavigator:
    def __init__(self):
        rospy.init_node('node_edge_navigator')

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
        self.nodes = {int(node.node): (node.x_pose, node.y_pose, node.z_pose, node.yaw) for node in ros_nodes}
        self.edges = [(int(edge.source), int(edge.target)) for edge in ros_edges]

        print("Nodes:", self.nodes)
        print("Edges:", self.edges)

        # Action client for move_base
        self.move_base_client = SimpleActionClient('/move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()    

        # Publisher for velocity commands
        self.velocity_publisher = rospy.Publisher('/tb_cmd_vel', Twist, queue_size=10)

        # Linear and angular velocities
        self.linear_velocity = 0.2  # meters per second
        self.angular_velocity = 0.5  # radians per second   

        # Initialize current and next nodes
        self.current_node = None
        self.next_node = None

        # Loop through edges and navigate
        for start_node, end_node in self.edges:
            #end_node = int(input(f"Enter the goal node: "))
            self.navigate_between_nodes(start_node, end_node)

        rospy.spin()

    def navigate_between_nodes(self, start_node, end_node):
        start_pose = self.nodes[start_node]
        end_pose = self.nodes[end_node]

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose = Pose(
            position=Point(x=end_pose[0], y=end_pose[1], z=end_pose[2]),
            orientation=Quaternion(0, 0, end_pose[3], 1)
        )

        rospy.loginfo(f"Navigating from Node {start_node} to Node {end_node}")
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        rospy.loginfo(f"Navigation to Node {end_node} complete")

        self.current_node = start_node
        self.next_node = end_node

if __name__ == '__main__':
    NodeEdgeNavigator()
