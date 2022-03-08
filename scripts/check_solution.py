import sys
import xml.etree.ElementTree as ET
import numpy as np

class Obstacle:
    def __init__(self, defaults, init_config, sections):
        vals = {"mspeed": 1.0}
        vals.update(defaults)
        vals.update(init_config)
        self.id = str(vals["id"])
        self.size = float(vals["size"])
        self.mspeed = float(vals["mspeed"])
        self.sections = []
        for s in sections:
            s.attrib["size"] = self.size
            self.sections.append(Section(s.attrib))

class Agent:
    def __init__(self, tree, defaults):
        init_config = tree.attrib
        vals = {
        "start.heading": 0.0,
        "goal.heading": -1.0,
        "rotationspeed": 1.0
        }
        vals.update(defaults)
        vals.update(init_config)
        self.id_num = int(vals["id"])
        self.start_i = int(vals["start.x"])
        self.start_j = int(vals["start.y"])
        self.start_heading = float(vals["start.heading"])
        self.goal_i = int(vals["goal.x"])
        self.goal_j = int(vals["goal.y"])
        self.goal_heading = float(vals["goal.heading"])
        self.size = float(vals["size"])
        self.rspeed = float(vals["rotationspeed"])
        self.mspeed = float(vals["movespeed"])

            
class Section:
    def speed(self):
        return (((self.j2-self.j1)**2+(self.i2-self.i1)**2)**0.5)/self.duration

    def __init__(self, ic):
        init_config = {
            "g1": -1,
            "g2": -1,
            "size": 1.0
        }
        init_config.update(ic)
        self.i1 = int(init_config["start.x"])
        self.j1 = int(init_config["start.y"])
        self.i2 = int(init_config["goal.x"])
        self.j2 = int(init_config["goal.y"])
        self.size = float(init_config["size"])
        self.g1 = float(init_config["g1"])
        self.g2 = float(init_config["g2"])
        self.duration = float(init_config["duration"])

def read_solution_path(tree):
    log = tree.find("log")
    agent = log.find("agent")
    solved_agent = Agent(agent, {})
    path = agent.find("path")
    solution_path = []
    for s in path.findall("section"):
        solution_path.append(Section(s.attrib))
    return solution_path, solved_agent

def read_dynamic_obstacles(tree):
    dynamicobstacles = tree.find("dynamicobstacles")
    try:
        default = dynamicobstacles.find("defaultparameters").attrib
    except:
        default = {}
    dos = []
    for obstacle in dynamicobstacles.findall("obstacle"):
        dos.append(
            Obstacle(default, obstacle.attrib, obstacle.findall("section"))
        )
    return dos

def agent_pos(solution, agent, t):
    time = 0.0
    for section in solution:
        if (time <= t) and (t <= time + section.duration):
            dx = ((section.i2 - section.i1)**2 + (section.j2 - section.j1)**2)**0.5
            movetime = dx/agent.mspeed
            waittime = section.duration - movetime
            if waittime <= 0.0:
                waittime = 0.0
            if movetime > 0.0:
                dt = (t - time - waittime)/movetime
                if dt < 0:
                    dt = 0
                i = (1.0 - dt)*section.i1 + dt*section.i2 
                j = (1.0 - dt)*section.j1 + dt*section.j2
            else:
                i = section.i1
                j = section.j1
            size = agent.size
            return i, j, size
        time += section.duration
    return solution[-1].i2, solution[-1].j2, agent.size

def obstacle_pos(obstacle, t):
    time = 0.0
    for section in obstacle.sections:
        if (time <= t) and (t <= time + section.duration):
            dt = (t - time)/section.duration
            i = (1.0 - dt)*section.i1 + dt*section.i2 
            j = (1.0 - dt)*section.j1 + dt*section.j2
            size = obstacle.size
            return i, j, size
        time += section.duration
    return obstacle.sections[-1].i2, obstacle.sections[-1].j2, obstacle.size

def safe(ap, op):
    dx = ((ap[0] - op[0])**2 + (ap[1] - op[1])**2)**0.5 - ap[2] - op[2]
    return dx


def check_solution(log, obstacles):
    print("Checking " + log)
    log = ET.parse(log)
    obs = ET.parse(obstacles)
    solution, agent = read_solution_path(log)
    obs = read_dynamic_obstacles(obs)

    total_time = 0.0
    for section in solution:
        total_time += section.duration
    print(total_time)
        
    for t in np.arange(0, total_time, 0.1):
        ap = agent_pos(solution, agent, t)
        for obstacle in obs:
            op = obstacle_pos(obstacle, t)
            dx = safe(ap, op)
            if dx < -0.0001:
                print("collision", t, obstacle.id, dx)
                print(ap)
                print(op)
