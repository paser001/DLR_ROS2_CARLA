import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from carla_parking_nodes.parking_slots import (
    build_parking_slots,
    get_slot_corners,
    make_staging_pose,
)


class ParkingMapNode(Node):
    def __init__(self):
        super().__init__('parking_map_node')

        self.marker_pub = self.create_publisher(MarkerArray, '/parking_map/markers', 10)
        self.slots = build_parking_slots()

        self.declare_parameter('show_staging_markers', True)
        self.show_staging_markers = bool(self.get_parameter('show_staging_markers').value)

        self.timer = self.create_timer(1.0, self.publish_markers)
        self.get_logger().info(f'Parking map node started with {len(self.slots)} slots')

    def publish_markers(self):
        marker_array = MarkerArray()
        marker_id = 0

        for slot in self.slots:
            # Slot outline
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

            # Slot label
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

            center = Marker()
            center.header.frame_id = 'map'
            center.header.stamp = self.get_clock().now().to_msg()
            center.ns = 'parking_center'
            center.id = marker_id
            marker_id += 1
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.scale.x = 0.25
            center.scale.y = 0.25
            center.scale.z = 0.25
            center.color.r = 0.0
            center.color.g = 0.7
            center.color.b = 1.0
            center.color.a = 1.0
            center.pose.position.x = float(slot["x"])
            center.pose.position.y = float(slot["y"])
            center.pose.position.z = float(slot["z"] + 0.2)

            marker_array.markers.append(center)

            if self.show_staging_markers:
                staging = make_staging_pose(slot, ego_y=None)

                staging_marker = Marker()
                staging_marker.header.frame_id = 'map'
                staging_marker.header.stamp = self.get_clock().now().to_msg()
                staging_marker.ns = 'parking_staging'
                staging_marker.id = marker_id
                marker_id += 1
                staging_marker.type = Marker.SPHERE
                staging_marker.action = Marker.ADD
                staging_marker.scale.x = 0.3
                staging_marker.scale.y = 0.3
                staging_marker.scale.z = 0.3
                staging_marker.color.r = 1.0
                staging_marker.color.g = 1.0
                staging_marker.color.b = 0.0
                staging_marker.color.a = 1.0
                staging_marker.pose.position.x = float(staging["x"])
                staging_marker.pose.position.y = float(staging["y"])
                staging_marker.pose.position.z = float(staging["z"] + 0.2)

                marker_array.markers.append(staging_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()