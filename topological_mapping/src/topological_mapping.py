#!/usr/bin/env python

import networkx as nx
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion

x_pose = 0
y_pose = 0
z_pose = 0
yaw = 0

def odom_callback(odom_msg):
    global odom, x_pose, y_pose, z_pose, yaw
    odom = odom_msg.pose.pose    
    x_pose = odom.position.x
    y_pose = odom.position.y
    z_pose = odom.position.z
    yaw = odom.orientation.z

def publish_nodes_as_markers(graph):
    marker_publisher = rospy.Publisher("/node_markers", Marker, queue_size=10)
    marker = Marker()
    marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    marker.header.frame_id = "map"  # Replace with desired frame ID
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 1.0

    for node, data in graph.nodes(data=True):
        p = Point()
        p.x = data['x_pose']  
        p.y = data['y_pose']
        p.z = data['z_pose']
        marker.points.append(p)

    # Publish the node markers
    marker_publisher.publish(marker)

def publish_edges_as_markers(graph):
    marker_publisher = rospy.Publisher("/edge_markers", Marker, queue_size=10)
    marker = Marker()
    marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    marker.header.frame_id = "map"  # Replace with desired frame ID
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.05  
    marker.color.a = 1.0
    marker.color.b = 1.0

    for u, v in graph.edges():
        p1 = Point()
        p1.x = graph.nodes[u]['x_pose'] 
        p1.y = graph.nodes[u]['y_pose']
        p1.z = graph.nodes[u]['z_pose']
        marker.points.append(p1)

        p2 = Point()
        p2.x = graph.nodes[v]['x_pose']
        p2.y = graph.nodes[v]['y_pose']
        p2.z = graph.nodes[v]['z_pose']
        marker.points.append(p2)

    # Publish the edge markers
    marker_publisher.publish(marker)
    
def graph_mapping():
    msg = """
Node indexing starts at 0, beginning with the current position of the robot!
n: add new node at the current position
e: add new edge between two nodes
p: print current map
s: save the current map to a file
q: quit the program
"""
    graph_map = nx.Graph()
    num_nodes = 0
    num_saves = 1
    run = True
    rospy.init_node('graph_mapping', anonymous=True)
    rospy.Subscriber("/tb_sim/odom", Odometry, odom_callback)
    x_temp = x_pose
    y_temp = y_pose
    z_temp = z_pose
    orientation = yaw
    graph_map.add_node(num_nodes, x_pose=x_temp, y_pose=y_temp, z_pose=z_temp, yaw=orientation)
    print(msg)
    while(run):
        key = input('Please input a command:\n')
        if key == 'n':
            num_nodes+=1
            x_temp = x_pose
            y_temp = y_pose
            z_temp = z_pose
            orientation = yaw
            graph_map.add_node(num_nodes, x_pose=x_temp, y_pose=y_temp, z_pose=z_temp, yaw=orientation)
            print('Number of nodes: ' + str(graph_map.number_of_nodes()))
            publish_nodes_as_markers(graph_map)

        elif key == 'e':
            node1 = input('Please input the first node:\n')
            node2 = input('Please input the second node:\n')
            graph_map.add_edge(int(node1),int(node2))
            publish_edges_as_markers(graph_map)
        elif key == 'p':
            print(graph_map.nodes(data=True))
            print(graph_map.edges(data=True))
        elif key == 's':
            nx.write_graphml(graph_map, "/home/aakash/ohmni_ws/src/topological_mapping/graph/topo"+str(num_saves)+".graphml", prettyprint=True)
        elif key == 'q':
            rospy.signal_shutdown('End of mapping.')
            run = False
 
            # # Extract node positions if available
            # node_positions = {node: (graph_map.nodes[node]['position'], graph_map.nodes[node]['orientation'])
            #                 for node in graph_map.nodes() if 'position' in graph_map.nodes[node]}

            # # Plot the graph
            # if len(node_positions) > 0:
            #     nx.draw(graph_map, pos=node_positions, with_labels=True, node_size=300, node_color='skyblue', font_weight='bold')
            # else:
            #     nx.draw(graph_map, with_labels=True, node_size=300, node_color='skyblue', font_weight='bold')

            # plt.title('Mapped Graph')
            # plt.show()
        else: 
            print('Please enter a valid command.\n')
    rospy.spin()
    

    
if __name__ == '__main__':
    graph_mapping()


