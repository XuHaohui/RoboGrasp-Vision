#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from robograsp_interfaces.srv import DetectObjects
from robograsp_interfaces.msg import PerceptionResult


class CameraInteractive(Node):
    def __init__(self):
        super().__init__('camera_interactive')

        self.detect_client_ = self.create_client(
            DetectObjects, '/perception/detect_objects')

        self.result_pub_ = self.create_publisher(
            PerceptionResult, '/perception/result', 10)

        self.get_logger().info('CameraInteractive ready')

    def scan(self):
        if not self.detect_client_.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('/perception/detect_objects service not available')
            return []

        req = DetectObjects.Request()
        future = self.detect_client_.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Service call failed')
            return []

        return future.result().results

    def publish(self, perception):
        msg = PerceptionResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = perception.header.frame_id
        msg.object_class = perception.object_class
        msg.confidence = perception.confidence
        msg.position = perception.position
        msg.bbox_size = perception.bbox_size
        msg.bbox_2d = perception.bbox_2d
        msg.sensor_frame = perception.sensor_frame
        msg.status = perception.status

        self.result_pub_.publish(msg)
        self.get_logger().info(
            f"Published '{msg.object_class}' at "
            f"({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})")


def main():
    rclpy.init(args=sys.argv)
    node = CameraInteractive()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            print("\nScanning...")
            results = node.scan()

            if not results:
                print("No objects detected.")
            else:
                print("\nDetected objects:")
                print("  #  class      position (x, y, z)            confidence")
                print("---  ---------  ---------------------------  ----------")
                for i, r in enumerate(results):
                    print(f"  {i + 1}:  {r.object_class:<9}  "
                          f"({r.position.x:.3f}, {r.position.y:.3f}, {r.position.z:.3f})    "
                          f"  {r.confidence:.2f}")

            user = input("\nSelect object number (or 'q' to quit): ").strip()

            if user.lower() == 'q':
                break

            try:
                idx = int(user) - 1
                if 0 <= idx < len(results):
                    node.publish(results[idx])
                    input("Press Enter for next round...")
                else:
                    print(f"Invalid number: {user}")
            except ValueError:
                print(f"Invalid input: {user}")

    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
