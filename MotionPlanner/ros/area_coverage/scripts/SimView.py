import imp
import rospy 
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from math import pi
robotName = {0:"uav", 1:"uav1", 2:"uav2"}

class QuadVizModel:
    q_incr = pi/8.0

    def __init__(self, id) -> None:
        name = robotName[id]
        self.quad = rospy.Publisher(f"{name}/joint_states", JointState, 1)
        self._name = name
        self.br_base = TransformBroadcaster()
        self.br_base_link = TransformBroadcaster()
        self._prop_ori = 0
        

    def publish_robot_state(self, pose:list):
        ts = TransformStamped()
        ts.header.stamp = rospy.Time.now()
        ts.header.frame_id = "world"
        ts.child_frame_id = f"{self._name}/base_link_inertia"

        ts.transform.translation.x = pose[0]
        ts.transform.translation.y = pose[1]
        ts.transform.translation.z = 2.0

        q = quaternion_from_euler(0, 0, pose[2])

        ts.transform.rotation.x = q[0]
        ts.transform.rotation.y = q[1]
        ts.transform.rotation.z = q[2]
        ts.transform.rotation.w = q[3]


        self.br_base.sendTransform(ts)
        self.br_base_link.sendTransform(ts)

        self.publish_propeller_spin()

    def publish_propeller_spin(self):
        msg = JointState()
        self._prop_ori += self.q_incr % 2 * pi 
        msg.header.stamp = rospy.Time.now()

        for i in range(4):
            rotor_name = f"rotor_{i}_joint"
            if i % 2 != 0:
                self._prop_ori *= -1 
            msg.position.append(self._prop_ori)
            msg.name.append(rotor_name)
        self.quad.publish(msg)

        


# import matplotlib.pyplot as plt
# import numpy as np
# import math
# def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
#     plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
#               head_length=width, head_width=width)
#     plt.plot(x, y)


# def plot_robot(x, y, yaw, robot_radius):  # pragma: no cover
#     circle = plt.Circle((x, y), robot_radius, color="b")
#     plt.gcf().gca().add_artist(circle)
#     out_x, out_y = (np.array([x, y]) +
#                     np.array([np.cos(yaw), np.sin(yaw)]) * robot_radius)
#     plt.plot([x, out_x], [y, out_y], "-k")


# class SimView(object):
#     def __init__(self, path ):
#         self.path = path

#     def update(self, robots):
#         plt.cla()
#         # for stopping simulation with the esc key.
#         plt.gcf().canvas.mpl_connect(
#             'key_release_event',
#             lambda event: [exit(0) if event.key == 'escape' else None])
#         self.show_trajectory(self.path)
#         for robot in robots:
#             plt.plot(robot.predicted_traj[:, 0], robot.predicted_traj[:, 1], "-g")
#             plt.plot(robot.x, robot.y, "xr", color='white')
#             N = len(robot.predicted_traj[:, 0])-1
#             dx = robot.predicted_traj[N, 0] - robot.x
#             dy = robot.predicted_traj[N, 1] - robot.y
#             yaw = math.atan2(dy, dx)
#             plot_arrow(robot.x, robot.y, yaw)
#             plot_robot(robot.x, robot.y, yaw, robot.radius/2-0.1)

#         #
#         plt.axis([-1, 5, -1, 6])
#         # plt.axis("equal")
#         plt.grid(True)
#         plt.pause(0.01)

#     def show_trajectory(self, trajectory):
#         plt.plot(trajectory[:, 0], trajectory[:, 1], "--k")

