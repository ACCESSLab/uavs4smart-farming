import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from pprint import pprint

class Rect:
    def __init__(self, origin, height, width):
        self.origin = origin
        self.height = height
        self.width = width

        self.A = np.array(origin)
        self.B = self.A + np.array([height, 0])
        self.C = self.A + np.array([height, width])
        self.D = self.A + np.array([0, width])

    def __repr__(self):
        return "({}) ({}) ({}) ({})".format(self.A, self.B, self.C, self.D)

def get_rectangle(rect):
    return Rectangle(rect.origin, rect.height, rect.width)

def decompose_line(start, end, delta = 0.2):
    theta = 0
    while(theta<=1):
        y = start*(1-theta) + end*theta
        theta += delta
        yield y

def get_sweep_path(rect):
    AB = decompose_line(rect.A, rect.B, delta = 0.1)
    DC = decompose_line(rect.D, rect.C, delta = 0.1)

    for i, (x,y) in enumerate(zip(AB, DC)):
        if i %2 == 0:
            yield x
            yield y
        else:
            yield y
            yield x

def RecursiveDecompose(path, result, delta = 2):
    manhatten = (np.array(path[0]) - np.array(path[1]))
    dist = 1.0*abs(manhatten[0]) + 0.5*abs(manhatten[1])
    dist1 = np.linalg.norm(np.array(path[0]) - np.array(path[1]))
    if(dist>delta and dist1> delta):
        y = (np.array(path[0])+ np.array(path[1]))/2.0
        path.insert(1, y.tolist())
        RecursiveDecompose(path, result, delta)
    else:
        result.append(path[0])
        path.remove(path[0])
        if len(path) >1 :RecursiveDecompose(path, result, delta)
        else: result.append(path[0])

def get_path():
    rect = Rect((0,0), 4, 5)
    path = get_sweep_path(rect)
    path = np.array(list(path))
    return path
if __name__ == '__main__':
    rect = Rect((0,0), 4, 5)
    path = get_sweep_path(rect)
    # pprint(list(path))


    path = np.array(list(path))
    traj = []
    RecursiveDecompose(path.tolist(), traj)
    traj = np.array(traj)
    fig, ax = plt.subplots()
    rectangle = get_rectangle(rect)
    rectangle.set_facecolor('white')
    rectangle.set_edgecolor('r')
    ax.add_patch(rectangle)
    ax.plot(path[:, 0], path[:, 1])
    ax.plot(traj[:, 0], traj[:, 1], '^')
    plt.axis([-1,8,-1,8])
    plt.show()