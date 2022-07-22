#python
import json 
import sys 
import os 
lib_path = "/home/redwan/PycharmProjects/MILP"
sys.path.append(lib_path)
from QuadMILP import Path, TrajManager


class QuadManager(TrajManager):
    def __init__(self):
        quadNames = ['/Quadcopter[%d]'%i for i in range(3)]
        targetNames = ['/target[%d]'%i for i in range(3)]
        self.quadHandles = [sim.getObject(quad) for quad in quadNames]
        self.targetHandles = [sim.getObject(target) for target in targetNames]
        self.altitude = self.getPositions()[0][-1]
    def getPositions(self):
        return [sim.getObjectPosition(handle, -1) for handle in self.quadHandles]
    def setInitPositions(self, point, index, init=True):
        pose = [point[0], point[1], self.altitude]
        if init:
            sim.setObjectPosition(self.quadHandles[index], -1, pose)
        sim.setObjectPosition(self.targetHandles[index], -1, pose)
        
    def setPositions(self, point, index):
        pose = [point[0], point[1], self.altitude]
        sim.setObjectPosition(self.targetHandles[index], -1, pose)
        
def sysCall_thread():
    # e.g. non-synchronized loop:
    filename = '/home/redwan/PycharmProjects/MILP/test/area_decomposition_02.json'
    with open(os.path.join(lib_path, filename)) as file:
        jdata = json.load(file)
    
    paths = []
    manager = QuadManager()
    for i in range(3):
        path = Path(index=i, data=jdata)
        path.fit(-5, 5, -5, 5)
        path.interpolate(velocity=0.1)
        # print(path)
        manager.setInitPositions(path[0], i)
        paths.append(path)
    sim.setThreadAutomaticSwitch(True)
    delay =0.2
    motion = manager.trajectory_planning(paths, delay)
    
    while True:
        for _ in range(3):
            result = next(motion)[0]
            print(result)
            manager.setInitPositions(result.positions, result.id, init=False)
        sim.wait(delay)
    