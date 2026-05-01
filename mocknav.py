import rclpy
from rclpy.node import Node
from bph_interfaces.srv import GoToLocation
rclpy.init()
n = Node('mock_nav')
n.create_service(GoToLocation, 'navigate_to',
    lambda req, resp: setattr(resp, 'accepted', True) or setattr(resp, 'message', 'ok') or resp)
rclpy.spin(n)
