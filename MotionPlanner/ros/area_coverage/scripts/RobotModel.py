import numpy as np
from pose_controller import PoseController
from SimView import QuadVizModel as viz 


class RobotModel(PoseController):
    T_MAX = 10
    def __init__(self, id, mass, V_MAX, F_MAX, dt, safe_dist = 0.55):
        # self.initialzed = False 
        # self._viz = viz(robotName[id])
        self.id = id
        self.m = mass
        self.V_MAX = V_MAX
        self.F_MAX = F_MAX
        self.V_MIN = -V_MAX
        self.F_MIN = -F_MAX
        self.dt = dt
        self.radius = safe_dist
        self.T_MAX = int(self.T_MAX/dt)
        self.set_state(0,0,0,0,0,0)
        self.predicted_traj = []
        self.count = 0

    def __sub__(self, other):
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
        
    def set_state(self, x, y, vx, vy, fx, fy):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.fx = fx
        self.fy = fy
        # super().__init__(init_pos=[x, y, 0], gains=[0.9, 1.5, 0.1], dt=0.03)
        # self.start()

    def publish_state(self, state: list):
        if not self.initialzed: return
        self.x = state[0]
        self.y = state[1]
        self._viz.publish_robot_state(state)

    def get_model(self):
        self.A = np.zeros([4, 4])
        self.B = np.zeros([4, 2])
        self.A[0][0] = 1
        self.A[1][1] = 1
        self.A[2][2] = 1
        self.A[3][3] = 1
        self.A[0][2] = self.dt
        self.A[1][3] = self.dt
        
        self.B[0][0] = (0.5 / self.m) * self.dt ** 2
        self.B[1][1] = (0.5 / self.m) * self.dt ** 2
        self.B[2][0] = self.dt / self.m 
        self.B[3][1] = self.dt / self.m
        
        return self.A, self.B

    def add_waypoints(self, waypoints):
        self.waypoints = waypoints
        self.x, self.y = waypoints[self.count]
        self.count = 1
        self.num_wp = len(self.waypoints) - 1

    def __repr__(self):
        return "> [robot {}]:({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f} )".format(self.id, self.x, self.y, self.vx, self.vy, self.fx, self.fy)
    def __next__(self):
        self.count = min(self.count, self.num_wp)
        return self.waypoints[self.count]

    def update(self, state, predicted_traj):
        self.predicted_traj = predicted_traj
        self.x, self.y, self.vx, self.vy, self.fx, self.fy = state
        current = np.array([self.x, self.y])
        # self.setGoal([self.x, self.y])
        # self.initialzed = True
        goal = self.waypoints[self.count]
        if(np.linalg.norm(goal - current) <= 0.50 and self.count < len(self.waypoints) ):
            self.count += 1
            self.count = min(self.count, self.num_wp)
            # print(f"robot {self.id} -> count {self.count} | total {len(self.waypoints)}")
        return self.count < len(self.waypoints)


