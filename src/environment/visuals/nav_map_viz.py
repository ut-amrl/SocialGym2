import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray

class NavMapViz:

    def __init__(self, nav_points, nav_lines):
        self.nav_map = rospy.Publisher("/visualization_marker_nav_map", Marker, queue_size=100)
        self.nav_map_titles = rospy.Publisher("/visualization_marker_nav_map_titles", MarkerArray, queue_size=100)

        self.nav_points = nav_points
        self.nav_lines = nav_lines

        self.marker = Marker()
        self.points = Marker()
        self.texts = MarkerArray()

        self.delete_markers()

        self.point([v for v in nav_points])
        self.text([v for v in nav_points])

    def delete_markers(self):
        p = Marker()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()
        p.action = Marker.DELETEALL
        self.points = p

        marker_array_msg = MarkerArray()
        marker_array_msg.markers.append(p)
        self.texts = marker_array_msg

        self.publish()


    def text(self, points):
        p = MarkerArray()

        for idx, point in enumerate(self.nav_points):
            a = Marker()
            # a.lifetime = 10
            a.header.frame_id = "map"
            a.header.stamp = rospy.Time.now()
            a.type = 9
            a.id = idx

            a.color.r = 1.0
            a.color.g = 0.0
            a.color.b = 0.53
            a.color.a = 0.5

            a.scale.x = 0.2
            a.scale.y = 0.2
            a.scale.z = 1.0

            a.pose.position.x = point[0] + 0.25
            a.pose.position.y = point[1] + 0.25
            a.pose.position.z = 1.0

            a.text = str(idx)

            p.markers.append(a)

        self.texts = p


    def point(self, points):
        p = Marker()
        p.header.frame_id = "map"
        p.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        p.type = 8
        p.id = 0
        # p.lifetime = 10

        p.color.r = 1.0
        p.color.g = 0.0
        p.color.b = 0.53
        p.color.a = 0.5

        p.scale.x = 0.2
        p.scale.y = 0.2
        p.scale.z = 1.0

        p.pose.position.x = 0
        p.pose.position.y = 0
        p.pose.position.z = 0

        pnts = []

        for point in points:
            a = Point()
            a.x = point[0]
            a.y = point[1]
            a.z = 1.0
            pnts.append(a)


        p.points = pnts

        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0

        self.points = p

    def rect(self, zone):
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self. marker.type = 1
        self.marker.id = 0

        # Set the scale of the marker
        self.marker.scale.x = abs(zone['upper_right'][0] - zone['upper_left'][0])
        self.marker.scale.y = abs(zone['upper_right'][1] - zone['lower_right'][1])
        self.marker.scale.z = 1.0

        # Set the color
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.5

        # Set the pose of the marker
        self.marker.pose.position.x = (zone['upper_right'][0] + zone['upper_left'][0])/2
        self. marker.pose.position.y = (zone['upper_right'][1] + zone['lower_right'][1])/2
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

    def publish(self):
        self.nav_map.publish(self.points)
        self.nav_map_titles.publish(self.texts)
