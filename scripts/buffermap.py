#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import binary_dilation, generate_binary_structure

def dilate_obstacles(map_data, radius_cells):
    """
    Expande áreas ocupadas e desconhecidas no mapa.
    """
    occupied = (map_data > 80) | (map_data == -1)  # 1 p/ ocupado ou desconhecido
    struct = generate_binary_structure(2, 2)       # conexões 4-vizinhos
    dilated = binary_dilation(occupied, structure=struct, iterations=radius_cells)
    return dilated.astype(np.int8)

class MapDilationNode:
    def __init__(self):
        rospy.init_node("map_dilation_node")

        self.buffer_radius_cells = rospy.get_param("~buffer_radius", 0)
        rospy.loginfo("Buffer radius (em células): %d", self.buffer_radius_cells)

        self.map_dilated_pub = rospy.Publisher("/map_dilated", OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

        rospy.loginfo("Aguardando mensagens em /map...")
        rospy.spin()

    def map_callback(self, msg):
        rospy.loginfo("Mapa recebido (%dx%d)", msg.info.width, msg.info.height)

        # Converte dados do mapa em matriz 2D
        map_array = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        # Aplica dilatação
        dilated_map = dilate_obstacles(map_array, self.buffer_radius_cells) * 100

        # Cria nova mensagem OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = msg.header.frame_id
        grid_msg.info = msg.info
        grid_msg.data = list(dilated_map.flatten())

        # Publica
        self.map_dilated_pub.publish(grid_msg)
        rospy.loginfo("Mapa dilatado publicado em /map_dilated")

if __name__ == "__main__":
    try:
        MapDilationNode()
    except rospy.ROSInterruptException:
        pass