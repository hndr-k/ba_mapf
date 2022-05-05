from tracemalloc import start
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateThroughPoses

from geometry_msgs.msg import PoseStamped
from mapf_actions.srv import Mapf


class MapfActionClient(Node):

    def __init__(self):
        super().__init__('navposes')
        self._action_client_1 = ActionClient(self, NavigateThroughPoses, '/robotino1/navigate_through_poses')
        self._action_client_2 = ActionClient(self, NavigateThroughPoses, '/robotino2/navigate_through_poses')
        self.client = self.create_client(Mapf, '/off_field/mapf_plan')
        self.path_rob1 = []
        self.path_rob2 = []
        self.req = Mapf.Request()

    def send_goal_1(self, path):
        goal_msg = NavigateThroughPoses.Goal()

 

       
        goal_msg.poses = path
        self._action_client_1.wait_for_server()
        
        return self._action_client_1.send_goal_async(goal_msg)
    def send_goal_2(self, path):
        goal_msg = NavigateThroughPoses.Goal()
       
    
        goal_msg.poses = path
        self._action_client_1.wait_for_server()
        
        return self._action_client_2.send_goal_async(goal_msg)
    
    def send_request_1(self):
        self.req.robotino_id = 0
        start_rob1 = PoseStamped()
        start_rob1.pose.position.x = 5.00
        start_rob1.pose.position.y = -2.00
        start_rob1.pose.position.z = 0.0
        start_rob1.pose.orientation.x = 0.0
        start_rob1.pose.orientation.y = 0.0
        start_rob1.pose.orientation.z = 0.0
        start_rob1.pose.orientation.z = 1.0
        start_rob1.header.frame_id = 'map'
        self.req.start = start_rob1
        
        
        goal_rob1 = PoseStamped()
        goal_rob1.pose.position.x = 5.00
        goal_rob1.pose.position.y = 2.00
        goal_rob1.pose.position.z = 0.0
        goal_rob1.pose.orientation.x = 0.0
        goal_rob1.pose.orientation.y = 0.0
        goal_rob1.pose.orientation.z = 0.0
        goal_rob1.pose.orientation.z = 1.0
        goal_rob1.header.frame_id = 'map'
        self.req.goal = goal_rob1
        self.req.timestamp = self.get_clock().now().to_msg()
        self.future = self.client.call_async(self.req)

    def send_request_2(self):
        self.req.robotino_id = 1
        start_rob2 = PoseStamped()
        start_rob2.pose.position.x = 5.00
        start_rob2.pose.position.y = 2.00
        start_rob2.pose.position.z = 0.0
        start_rob2.pose.orientation.x = 0.0
        start_rob2.pose.orientation.y = 0.0
        start_rob2.pose.orientation.z = 0.0
        start_rob2.pose.orientation.z = 1.0
        start_rob2.header.frame_id = 'map'
        self.req.start = start_rob2
        
        
        goal_rob1 = PoseStamped()
        goal_rob1.pose.position.x = 5.00
        goal_rob1.pose.position.y = -2.00
        goal_rob1.pose.position.z = 0.0
        goal_rob1.pose.orientation.x = 0.0
        goal_rob1.pose.orientation.y = 0.0
        goal_rob1.pose.orientation.z = 0.0
        goal_rob1.pose.orientation.z = 1.0
        goal_rob1.header.frame_id = 'map'
        self.req.goal = goal_rob1
        self.req.timestamp = self.get_clock().now().to_msg()
        self.future = self.client.call_async(self.req)

        



def main(args=None):
    rclpy.init(args=args)

    action_client = MapfActionClient()

    #future_1 = action_client.send_request_1()

    #while rclpy.ok():
    #    rclpy.spin_once(action_client)
    #    if action_client.future.done():
    #        try:
    #            response = action_client.future.result()
    #        except Exception as e:
    #            action_client.get_logger().info(
    #                'Service call failed %r' % (e,))
    #        else:
    #            action_client.get_logger().info(
    #                'Robotino Id %d' %
    #                (action_client.req.robotino_id))
    #            for msg in response.path.poses:
    #                action_client.get_logger().info('x : %f' % (msg.pose.position.y))
    #            
    #        break
    future_2 = action_client.send_request_2()

    while rclpy.ok():
        rclpy.spin_once(action_client)
        if action_client.future.done():
            try:
                response = action_client.future.result()
            except Exception as e:
                action_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                action_client.get_logger().info(
                    'Robotino Id %d' %
                    (action_client.req.robotino_id))
                action_client.path_rob2 = response.path.poses
                for msg in response.path.poses:
                    action_client.get_logger().info('x : %f y : %f' % (msg.pose.position.x, msg.pose.position.y))
                
            break
    #future_1 = action_client.send_request_1()
    #response.path.poses.clear()
    #while rclpy.ok():
    #    rclpy.spin_once(action_client)
    #    if action_client.future.done():
    #        try:
    #            response = action_client.future.result()
    #        except Exception as e:
    #            action_client.get_logger().info(
    #                'Service call failed %r' % (e,))
    #        else:
    #            action_client.get_logger().info(
    #                'Robotino Id %d' %
    #                (action_client.req.robotino_id))
    #            action_client.path_rob1 = response.path.poses
    #            for msg in response.path.poses:
    #                action_client.get_logger().info('x : %f y : %f' % (msg.pose.position.x, msg.pose.position.y))
                
     #       break
    #future_2 = action_client.send_request_2()

    while rclpy.ok():
        rclpy.spin_once(action_client)
        if action_client.future.done():
            try:
                response = action_client.future.result()
            except Exception as e:
                action_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                action_client.get_logger().info(
                    'Robotino Id %d' %
                    (action_client.req.robotino_id))
                action_client.path_rob2 = response.path.poses
                for msg in response.path.poses:
                    action_client.get_logger().info('x : %f' % (msg.pose.position.y))
                
            break
    action_client.send_goal_1(action_client.path_rob1)
    action_client.send_goal_2(action_client.path_rob2)
    action_client.destroy_node()


if __name__ == '__main__':
    main()