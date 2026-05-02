import rclpy
from rclpy.node import Node

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.get_logger().info('Path Planner node started!')
        
        # A timer that runs every 1 second
        self.timer = self.create_timer(1.0, self.plan_path)
    
    def plan_path(self):
        self.get_logger().info('Planning a path...')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)          # keeps the node alive
    rclpy.shutdown()

if __name__ == '__main__':
    main()