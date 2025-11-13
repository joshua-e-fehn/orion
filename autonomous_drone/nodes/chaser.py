import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode

class Chaser(Node):
    def __init__(self):
        super().__init__('chaser')
        ns = self.declare_parameter('namespace', 'drone1').value
        target_ns = self.declare_parameter('target_namespace', 'drone2').value
        self.max_speed = float(self.declare_parameter('max_speed', 1.0).value)
        self.kp = float(self.declare_parameter('kp', 0.8).value)

        self.own_pose_topic = f'/{ns}/mavros/local_position/pose'
        self.target_pose_topic = f'/{target_ns}/mavros/local_position/pose'
        self.cmd_vel_topic = f'/{ns}/mavros/setpoint_velocity/cmd_vel_unstamped'

        self.own_pose = None
        self.target_pose = None

        self.create_subscription(PoseStamped, self.own_pose_topic, self.cb_own, 10)
        self.create_subscription(PoseStamped, self.target_pose_topic, self.cb_target, 10)
        self.pub_vel = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        self.arm_client = self.create_client(CommandBool, f'/{ns}/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, f'/{ns}/mavros/set_mode')

        self.offboard_sent = False
        self.armed = False

        self.timer = self.create_timer(0.1, self.timer_cb)

    def cb_own(self, msg):
        self.own_pose = msg

    def cb_target(self, msg):
        self.target_pose = msg

    def try_arm_and_offboard(self):
        # attempt services if available
        if not self.arm_client.wait_for_service(timeout_sec=0.5):
            return
        if not self.set_mode_client.wait_for_service(timeout_sec=0.5):
            return

        if not self.offboard_sent:
            req = SetMode.Request()
            try:
                req.custom_mode = 'OFFBOARD'
            except Exception:
                pass
            fut = self.set_mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
            self.offboard_sent = True

        if not self.armed:
            req = CommandBool.Request()
            req.value = True
            fut = self.arm_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
            try:
                self.armed = bool(fut.result().success)
            except Exception:
                self.armed = False

    def timer_cb(self):
        self.try_arm_and_offboard()
        if self.own_pose is None or self.target_pose is None:
            return

        ox = self.own_pose.pose.position.x
        oy = self.own_pose.pose.position.y
        oz = self.own_pose.pose.position.z
        tx = self.target_pose.pose.position.x
        ty = self.target_pose.pose.position.y
        tz = self.target_pose.pose.position.z

        dx = tx - ox
        dy = ty - oy
        dz = tz - oz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist < 0.2:
            vx = vy = vz = 0.0
        else:
            vx = self.kp * dx
            vy = self.kp * dy
            vz = self.kp * dz
            speed = math.sqrt(vx*vx + vy*vy + vz*vz)
            if speed > self.max_speed:
                s = self.max_speed / speed
                vx *= s; vy *= s; vz *= s

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.pub_vel.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Chaser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()