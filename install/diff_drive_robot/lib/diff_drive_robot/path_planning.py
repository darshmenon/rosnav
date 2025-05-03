import heapq
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker

class ImprovedAStar(Node):
    def __init__(self):
        super().__init__('improved_astar')
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.marker_pub = self.create_publisher(Marker, 'obstacle_markers', 10)
        
        # Configurable parameters
        self.declare_parameter('grid_size_x', 20)
        self.declare_parameter('grid_size_y', 20)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('safety_margin', 0.3)
        
        # Convert world coordinates to grid
        self.obstacles = self.process_obstacles([(2,1), (4,3), (5,1)])
        self.publish_obstacles()
        
        self.create_timer(1.0, self.plan_path)

    def process_obstacles(self, raw_obstacles):
        """Convert real-world coordinates to grid coordinates"""
        res = self.get_parameter('resolution').value
        return [(int(x/res), int(y/res)) for x,y in raw_obstacles]

    def heuristic(self, a, b):
        # Euclidean distance for better performance
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def plan_path(self):
        start = self.world_to_grid(0, 0)
        goal = self.world_to_grid(8, 8)
        
        path = self.astar(start, goal)
        if path:
            self.publish_path(path)

    def astar(self, start, goal):
        # Improved version with 8-direction movement
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == goal:
                return self.reconstruct_path(came_from, current)

            # 8-direction movement
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1),
                         (-1,-1), (-1,1), (1,-1), (1,1)]:
                neighbor = (current[0]+dx, current[1]+dy)
                
                if self.valid_grid_position(neighbor):
                    temp_g = g_score[current] + np.sqrt(dx**2 + dy**2)
                    
                    if neighbor not in g_score or temp_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = temp_g
                        f_score[neighbor] = temp_g + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    def valid_grid_position(self, pos):
        grid_x = self.get_parameter('grid_size_x').value
        grid_y = self.get_parameter('grid_size_y').value
        margin = int(self.get_parameter('safety_margin').value / 
                   self.get_parameter('resolution').value)
                   
        return (0 <= pos[0] < grid_x and 
                0 <= pos[1] < grid_y and 
                all((abs(pos[0]-o[0]) > margin or 
                    abs(pos[1]-o[1]) > margin for o in self.obstacles)))

    def publish_path(self, path):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        for grid_pos in path:
            pose = PoseStamped()
            pose.pose.position = self.grid_to_world(*grid_pos)
            msg.poses.append(pose)
            
        self.path_pub.publish(msg)

    # Add visualization and coordinate conversion methods