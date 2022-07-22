#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt


class PathReader:
    def __init__(self, index, data=None, **kwargs):
        self.index = index
        self.__key = "%d" % index
        if data is not None:
            self(data, kwargs)

    def __call__(self, item, kwargs):

        if 'initial_positions' in kwargs:
            n = kwargs['initial_positions']
            initX = [item["initial_positions"]["x"][i] for i in range(n)]
            initY = [item["initial_positions"]["y"][i] for i in range(n)]
            self.__data = np.array(list(zip(initX, initY)))
        else:
            X = item["paths"][self.__key]['x']
            Y = item["paths"][self.__key]['y']
            self.__data = np.array(list(zip(X, Y)))



    def __len__(self):
        return len(self.__data)

    def __repr__(self):
        return f"[data: {self.index}] \n {self.__data}"

    def plot(self):
        print(self.__data.shape)
        plt.plot(self.__data[:, 0], self.__data[:, 1])

    def scatter(self):
        plt.scatter(self.__data[:, 0], self.__data[:, 1])

    def scale(self, x, x_max, x_min):
        X_std = x / 100.0
        X_scaled = X_std * (x_max - x_min) + x_min
        return X_scaled

    def fit(self, x_min, x_max, y_min, y_max):
        for i, d in enumerate(self.__data):
            x, y = d
            self.__data[i][0] = self.scale(x, x_max, x_min)
            self.__data[i][1] = self.scale(y, y_max, y_min)

    def interpolate(self, velocity=0.5):
        path = self.__data.copy()
        N = len(path)
        self.__data = None
        for i in range(1, N):
            d = np.linalg.norm(path[i - 1] - path[i])
            num_points = int(d / velocity)
            x = np.linspace(path[i - 1][0], path[i][0], num_points)
            y = np.linspace(path[i - 1][1], path[i][1], num_points)
            data = np.array([x, y]).T
            if len(data):
                self.__data = data if self.__data is None else np.vstack((self.__data, data))
                # print(data.shape)

    def __getitem__(self, item):
        assert item < len(self)
        return self.__data[item]

    @property
    def toPoseArray(self):
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = str(self.index)
        for pos in self.__data:
            pose = Pose()
            pose.position.x = pos[0]
            pose.position.y = pos[1]
            msg.poses.append(pose)
        return msg

def publishMsg(pub, paths:PathReader):
    rospy.loginfo("publishing paths")
    for path in paths:
        time.sleep(0.1)
        pub.publish(path.toPoseArray)
if __name__ == '__main__':
    import json
    import os
    import time
    import sys
    import rospy
    from geometry_msgs.msg import PoseArray, Pose, Point


    rospy.init_node('path_reader', disable_signals=True)

    assert len(sys.argv) > 1, "missing json file"

    # take a pause before decoding the file
    time.sleep(5)
    filename = sys.argv[1]
    with open(filename) as file:
        jdata = json.load(file)

    rospy.loginfo('path reader node started')
    # assign geometric path to each robot with header frame_id
    paths = [PathReader(index=i, data=jdata) for i in range(3)]
    topics = ['/multiquad/tasks', '/multiquad/exec']
    pubs =  [rospy.Publisher(topic, PoseArray, queue_size=10) for topic in topics]
    publishMsg(pubs[0], paths)
    time.sleep(1)

    # initial positions
    rospy.loginfo("publishing executing commands")
    init = PathReader(index=4, data=jdata, initial_positions=3)
    pubs[1].publish(init.toPoseArray)
    time.sleep(1)
    rospy.loginfo("path reader node terminated")

