import rospy
from geometry_msgs.msg import PoseArray, Pose, Point
from SweepPath import RecursiveDecompose
import numpy as np
import heapq






class candidate_solution:
    def __init__(self, point, line):
        self.point = point
        self.line = line
    def get_area(self):
        return self.triangle_area(self.point, self.line)

    @staticmethod
    def triangle_area(point, line):
        def area(tri):
            x1, y1, x2, y2, x3, y3 = tri[0][0], tri[0][1], tri[1][0], tri[1][1], tri[2][0], tri[2][1]
            return abs(0.5 * (((x2 - x1) * (y3 - y1)) - ((x3 - x1) * (y2 - y1))))

        triangle = line + [tuple(point)]
        return area(triangle)
    def to_triangle(self):
        triangle = [tuple(self.point)] + self.line + [tuple(self.point)]
        return np.array(triangle)

    def __lt__(self, other):
        return self.get_area() < other.get_area()

    def __eq__(self, other):
        same_point = lambda x, y :(x[0] == y[0]) and  (x[1] == y[1])
        same_lines = []
        for x, y in zip(self.line, other.line):
            same_lines.append(same_point(x, y))
        return same_point(self.point, other.point) or all(same_lines)

    def __repr__(self):
        return "[area ]:= {:.3f} | [point ]:= {}, | [line ]:= {}".format(self.get_area(), self.point, self.line)


def min_max_scalar(X):
    X_min = 0
    X_max = 100
    Y_min = -5
    Y_max = 5
    Factor = (X - X_min) / (X_max - X_min)
    Y = Factor * (Y_max - Y_min) + Y_min
    return Y

class TaskAllocator:
    def __init__(self, motion_planner):
        self.count = 0
        self.motion_planner = motion_planner
        self.task_sub = rospy.Subscriber("/multiquad/tasks", PoseArray, self.callback)
        self.exec_sub = rospy.Subscriber("/multiquad/exec", PoseArray, self.execute)
        self.task_queue = {}
        self.altitude = rospy.get_param('~altitude', 1.0)   # altitude
        self.interpolationDist = rospy.get_param('~interpolation', 1.0)
        self.pub_paths = rospy.Publisher('/multiquad/paths', PoseArray, queue_size=1000)

    def callback(self, data):
        id = int(data.header.frame_id)
        # rospy.loginfo(id)
        path = []
        for pose in data.poses:
            point = [pose.position.x, pose.position.y]
            path.append(point)
        self.task_queue[id] = self.transform(path)
        self.publish_path(self.task_queue[id], id)


    def execute(self, data):
        """
        @param data: contains initial poses
        """
     
        rospy.loginfo("publishing ....")
        # rospy.loginfo(data)
        final_traj = self.task_queue.values()
        all_lines = [ [t[0], t[-1]] for t in final_traj]
        init_points = map(lambda quad:[min_max_scalar(quad.position.x), min_max_scalar(quad.position.y)], data.poses)
        init_points = list(init_points)
        h = []
        for line in all_lines:
            for p in init_points:
                candidate = candidate_solution(p, line)
                heapq.heappush(h, candidate)
        sol = []
        while(len(sol) < len(init_points)):
            candidate = heapq.heappop(h)
            if not candidate in sol:
                sol.append(candidate)


        final_traj, traj_len = [], []
        for candidate, path in zip(sol, self.task_queue.values()):

            # import sys
            # import os 
            # sys.path.append(os.path.dirname(__file__))
            # from PyTraj import PyTraj
            # complete_path = [candidate.point] + path + [candidate.point]
            # wp_x = [x[0] for x in complete_path]
            # wp_y = [x[1] for x in complete_path]
            # traj = PyTraj(0.35, 0.35)
            # traj.set(wp_x, wp_y)
            # init_traj = [traj[i][1:-1] for i in range(len(traj))]

            ####################
            # connect initial robot position to first waypoint 
            init_path = [candidate.point, path[0]]
            # connect last robot position to initial robot position 
            last_path = [path[-1], candidate.point]
            init_traj, last_traj = [], []
            RecursiveDecompose(init_path,init_traj, self.interpolationDist)
            RecursiveDecompose(last_path,last_traj, self.interpolationDist)
            print('path length', len(path), 'init_traj', len(init_traj), 'last_traj', len(last_traj),  end=" ")
            init_traj += list(path) + last_traj
            print("total length", len(init_traj))
            final_traj.append(init_traj)
            traj_len.append(len(init_traj))

        # make all trajectories to the same length
        require_len = max(traj_len)

        for i, traj in enumerate(final_traj):
            N = len(traj)
            delta = require_len - N
            print('require length', require_len, 'delta', delta, end=" ")
            if N < require_len:
                final_traj[i] += [traj[-1]]*delta # repeating the last element
            print("modified length", len(final_traj[i]))


        self.motion_planner.trajectory_planning(final_traj)

    def transform(self, path):
        if len(path) < 2:
            rospy.loginfo('Empty Trajectory found')
            return None
        data = map(lambda x:[min_max_scalar(x[0]), min_max_scalar(x[1])], path)
        result = []
        RecursiveDecompose(list(data),result, self.interpolationDist)
        return result

    def publish_path(self, path, index):
        path_msg = PoseArray()
        path_msg.header.frame_id = str(index)
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.seq = index
        for point in path:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = self.altitude
            pose = Pose()
            pose.position = p
            path_msg.poses.append(pose)
        r = rospy.Rate(10)
        while True:
            self.pub_paths.publish(path_msg)
            r.sleep()
            if self.pub_paths.get_num_connections() > 0: break