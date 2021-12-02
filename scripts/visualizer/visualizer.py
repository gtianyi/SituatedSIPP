#!/usr/bin/env python3
import numpy as np
import xml.etree.ElementTree as ET
import pygame
import sys
import datetime
import argparse

BLACK = (0, 0, 0)
GREY = (128, 128, 128)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GOLD = (255, 215, 0)

pixel_factor = 1
class World:
    def __init__(self, tree, online, maptree, obstree):
        self.map = StaticMap(maptree)
        #self.agent_list = Agent.read_agents(tree)
        self.dynamic_obstacles = read_dynamic_obstacles(obstree)
        self.solved_agent, self.solution_path = read_solution_path(tree)
        self.dynamic_objects = [self.solution_path]
        self.sizes = [self.solved_agent.size]
        for obstacle in self.dynamic_obstacles:
            self.sizes.append(obstacle.size)
            self.dynamic_objects.append(obstacle.sections)
        self.goal = self.solved_agent.render_goal()
        self.online = online
        if self.online:
            self.online_soln_paths, self.online_soln_paths_t = read_online_paths(tree)
        self.screen = None
        self.background = None

    def render_agent(self, agent, agent_radius, paths, time, path_start_time = 0.0):
        # agent is not actually needed rn
        size = self.screen.get_rect()
        bgr = self.background.get_rect()
        scale = [size.width/bgr.width, size.height/bgr.height]
        surf = pygame.Surface((size.width, size.height), pygame.SRCALPHA, 32)
        c = {
            "Agent": GOLD,
            "Obstacle": RED
        }
        acc = path_start_time
        for section in paths:
            acc += section.duration
            if acc >= time:
                break
        if acc < time:
            time = acc
        sections = []
        acc2 = path_start_time
        for section in paths:
            acc2 += section.duration
            if acc2 >= time:
                sections.append(section)

        section_start_time = acc - sections[0].duration
        time_in_section = time - section_start_time
        dx = sections[0].i2 - sections[0].i1
        dy = sections[0].j2 - sections[0].j1
        x = sections[0].i1 + (time_in_section/sections[0].duration)*dx
        y = sections[0].j1 + (time_in_section/sections[0].duration)*dy
        rect = pygame.Rect(round(scale[0]*(x + (1/2 - agent_radius))) , round(scale[1]*(y + (1/2-agent_radius) )), scale[0]*2*agent_radius, scale[1]*2*agent_radius)
        pygame.draw.ellipse(surf, c[agent], rect)
        points = [(scale[0]*(x + 1/2), scale[1]*(y + 1/2)), (scale[0]*(sections[0].i2 + 1/2), scale[1]*(sections[0].j2 + 1/2))]
        for section in sections[1:]:
            points.append((scale[0]*(section.i1 + 1/2), scale[1]*(section.j1 + 1/2)))
            points.append((scale[0]*(section.i2 + 1/2), scale[1]*(section.j2 + 1/2)))
        pygame.draw.aalines(surf, c[agent], False, points)
        return surf

    def render_all(self, time):
        bg = self.background.copy()
        bg.blit(self.goal[0], self.goal[1])
        self.screen.blit(pygame.transform.scale(bg, self.screen.get_rect().size), (0, 0))
        for i in range(len(self.dynamic_objects)):
            p = self.dynamic_objects[i]
            if i == 0:
                if self.online and time < self.online_soln_paths_t[-1]:
                    j = 0
                    while self.online_soln_paths_t[j] < time:
                        j += 1
                    if j == 0:
                        path_start_time = 0.0
                    else:
                        path_start_time = self.online_soln_paths_t[j-1]
                    a_surf = self.render_agent("Agent", self.sizes[i], self.online_soln_paths[j], time, path_start_time)
                else:
                    a_surf = self.render_agent("Agent", self.sizes[i], p, time)
            else:
                a_surf= self.render_agent("Obstacle", self.sizes[i], p, time)
            self.screen.blit(a_surf, (0, 0))
        pygame.display.update()

    def start(self):
        # create a surface on screen that has the size of 240 x 180
        pygame.init()
        self.screen = pygame.display.set_mode((pixel_factor*self.map.width(),pixel_factor*self.map.height()), pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE)
        self.screen.fill("grey")
        self.background, _ = self.map.render()
        self.screen.blit(pygame.transform.scale(self.background, self.screen.get_rect().size), (0, 0))
        pygame.display.update()
        # define a variable to control the main loop
        running = True
        clock = pygame.time.Clock()
        time_to_done = get_paths_done_time(self.solution_path)
        self.start_time = datetime.datetime.now()
        time = 0.0
        # main loop
        while running:
            clock.tick(60)
            #print(update_fps(clock))
            # event handling, gets all event from the event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.display.quit()
                    sys.exit()
                elif event.type == pygame.VIDEORESIZE:
                    self.screen = pygame.display.set_mode(event.size, pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE)
            time = (datetime.datetime.now() - self.start_time).total_seconds()
            #print("\r" + str(time), end = "")
            self.render_all(time)


class StaticMap:
    def __init__(self, tree):
        m = tree.find("map")
        grid = m.find("grid")
        self.map_array = np.zeros((int(grid.attrib["height"]),
                              int(grid.attrib["width"])), dtype=int)
        acc = 0
        for row in grid.findall("row"):
            self.map_array[acc] = row.text.strip().split(" ")
            acc += 1

    def shape(self):
        return self.map_array.shape

    def width(self):
        return self.shape()[1]
    def height(self):
        return self.shape()[0]

    def render(self):
        background = pygame.Surface((self.width(), self.height()))
        for i in range(self.height()):
            for j in range(self.width()):
                rect = pygame.Rect(j, i, 1, 1)
                if self.map_array[i, j] == 1:
                    pygame.draw.rect(background, BLACK, rect, 0)
                else:
                    pygame.draw.rect(background, WHITE, rect, 0)
        return background, rect

class Agent:
    @staticmethod
    def read_agents(tree):
        agents = tree.find("agents")
        agent_list = []
        try:
            default = agents.find("defaultparameters").attrib
        except:
            default = {}
        for a in agents.findall("agent"):
            agent_list.append(Agent(a, default))
        return agent_list

    def render_goal(self):
        surf = pygame.Surface((1, 1))
        rect = pygame.Rect(self.goal_i, self.goal_j, 1, 1)
        pygame.draw.rect(surf, "green", pygame.Rect(0,0,1,1), 0)
        return surf, rect

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


class Obstacle:
    def __init__(self, defaults, init_config, sections):
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




def get_paths_done_time(paths):
    acc = 0.0
    for section in paths:
        acc += section.duration
    return acc

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

def read_online_paths(tree):
    log = tree.find("log")
    agent = log.find("agent")
    paths = agent.find("onlineplanpaths")
    online_paths = []
    opd = []
    for path in paths.findall("path"):
        solution_path = []
        opd.append(float(path.attrib["duration"]))
        for s in path.findall("section"):
            solution_path.append(Section(s.attrib))
        online_paths.append(solution_path)
    opd = np.asarray(opd)
    opd[0:-1] = opd[0:-1] - opd[1:]
    return online_paths, np.cumsum(opd)

def read_solution_path(tree):
    log = tree.find("log")
    agent = log.find("agent")
    solved_agent = Agent(agent, {})
    path = agent.find("path")
    solution_path = []
    for s in path.findall("section"):
        solution_path.append(Section(s.attrib))
    return solved_agent, solution_path

def update_fps(clock):
	fps = str(int(clock.get_fps()))
	return fps

def parse_arguments():
    parser = argparse.ArgumentParser(description='Visualize task logs.')
    parser.add_argument("tree", help="task log xml file")
    parser.add_argument("--online", action='store_true')
    parser.add_argument("--map", default = "")
    parser.add_argument("--obs", default = "")
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    tree = ET.parse(args.tree)
    if args.map == "":
        maptree = tree
    else:
        maptree = ET.parse(args.map)
    if args.obs == "":
        obstree = tree
    else:
        obstree = ET.parse(args.obs)
    world = World(tree, args.online, maptree, obstree)
    world.start()
