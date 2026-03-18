import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from carla_parking_nodes.parking_slots import build_parking_slots, get_slot_corners


class ParkingMapNode(Node):
    def __init__(self):
        super().__init__('parking_map_node')

        self.marker_pub = self.create_publisher(MarkerArray, '/parking_map/markers', 10)
        self.slots = build_parking_slots()

        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f'Parking map node started with {len(self.slots)} slots')

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        for slot in self.slots:
            outline = Marker()
            outline.header.frame_id = 'map'
            outline.header.stamp = self.get_clock().now().to_msg()
            outline.ns = 'parking_outline'
            outline.id = marker_id
            marker_id += 1
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.scale.x = 0.08
            outline.color.r = 0.0
            outline.color.g = 1.0
            outline.color.b = 0.0
            outline.color.a = 1.0

            for x, y, z in get_slot_corners(slot):
                p = Point()
                p.x = float(x)
                p.y = float(y)
                p.z = float(z)
                outline.points.append(p)

            marker_array.markers.append(outline)

            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'parking_text'
            text.id = marker_id
            marker_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.scale.z = 0.6
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.pose.position.x = float(slot["x"])
            text.pose.position.y = float(slot["y"])
            text.pose.position.z = float(slot["z"] + 0.6)
            text.text = slot["id"]

            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()