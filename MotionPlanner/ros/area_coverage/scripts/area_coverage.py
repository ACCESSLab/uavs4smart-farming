#!/usr/bin/env python3
from RobotModel import RobotModel
from TrajPlanner import Solver
import rospy
import os, time, math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from TaskAllocator import TaskAllocator


class MotionPlanner:
    def __init__(self):
        self.pub_trajs = rospy.Publisher('/multiquad/trajs', JointTrajectory, queue_size=1000)

    def get_yaw_angle(self, predicted_traj):
        first_point = predicted_traj[0]
        last_point = predicted_traj[-1]
        dx = last_point[0] - first_point[0]
        dy = last_point[1] - first_point[1]
        yaw = math.atan2(dy, dx)
        return yaw



    def gen_traj_msg(self, robot, seq):
        traj = JointTrajectory()
        traj.header.frame_id = '%s'%robot.id
        traj.header.stamp = rospy.Time.now()
        traj.header.seq = seq
        # traj.joint_names = ['base_link_inertia', 'rotor_0_joint', 'rotor_1_joint', 'rotor_2_joint', 'rotor_3_joint']
        duration = 0
        yaw = self.get_yaw_angle(robot.predicted_traj)
        for state in robot.predicted_traj:
            # rospy.loginfo(state)
            s = JointTrajectoryPoint()
            s.positions = state[0:2]
            s.velocities = state[2:4]
            s.accelerations = [f/robot.m for f in state[4:6]]
            s.effort = [yaw]
            duration += robot.dt
            s.time_from_start = rospy.Duration(duration)
            traj.points.append(s)
        return traj

    def isRunning(self, robots):
        status = (robots[i].count < len(robots[i].waypoints) for i in range(len(robots)))
        return any(status)

    def trajectory_planning(self, all_paths):
        rospy.loginfo("Trajecotry planning ....")
        robots = []
        for i, wp in enumerate(all_paths):
            robot = RobotModel(id=i, dt= dt, mass=MASS, V_MAX=V_MAX, F_MAX=F_MAX, safe_dist=safe_dist)
            robot.add_waypoints(wp)
            robots.append(robot)
        rospy.loginfo("Trajecotry following .... (%d)"%len(robots[0].waypoints))
        wait = rospy.Rate(5)

        while self.isRunning(robots):
            try:
                solver = Solver(robots)
                if solver.solve():
                    for count, r in enumerate(robots):
                        traj = self.gen_traj_msg(r, count)
                        self.pub_trajs.publish(traj)
                wait.sleep()
            except KeyboardInterrupt:
                break



if __name__ == '__main__':
    time.sleep(2)
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name, disable_signals=True)
    rospy.loginfo('Starting [%s] node' % node_name)
    MASS = rospy.get_param('~mass', 1.8)
    V_MAX = rospy.get_param('~v_max', 0.3375)  # m/s
    F_MAX = rospy.get_param('~f_max', 0.441)  # N
    dt = rospy.get_param('~dt', 0.5)   # sampling time
    altitude = rospy.get_param('~altitude', 1.0)   # altitude
    safe_dist = rospy.get_param('~safe_dist', 0.75)   # altitude
    mp = MotionPlanner()
    allocation = TaskAllocator(mp)
    rospy.spin()