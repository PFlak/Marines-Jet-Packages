import rclpy
from narval_thruster_manager.thrusterManager import ThrusterManager

def main():
    rclpy.init()

    node = rclpy.create_node("n_thruster_manager")

    thrusterManager = ThrusterManager(node)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()