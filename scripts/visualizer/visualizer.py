#!/usr/bin/env python3
import numpy as np
import xml.etree.ElementTree as ET
import pygame
import sys
import datetime

BLACK = (0, 0, 0)
GREY = (128, 128, 128)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GOLD = (255, 215, 0)

pixel_factor = 200

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
        default = agents.find("defaultparameters").attrib
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


def render_agent(agent, paths, time):
    # agent is not actually needed rn
    surf = pygame.Surface((1, 1))
    acc = 0.0
    c = {
        "Agent": GOLD,
        "Obstacle": RED
    }
    for section in paths:
        acc += section.duration
        if acc >= time:
            break
    if acc < time:
        time = acc
    section_start_time = acc - section.duration
    time_in_section = time - section_start_time
    dx = section.i2 - section.i1
    dy = section.j2 - section.j1
    x = section.i1 + (time_in_section/section.duration)*dx
    y = section.j1 + (time_in_section/section.duration)*dy
    rect = pygame.Rect(x, y, 1, 1)
    pygame.draw.rect(surf, c[agent], pygame.Rect(0,0,1,1), 0)
    return surf, rect

def get_paths_done_time(paths):
    acc = 0.0
    for section in paths:
        acc += section.duration
    return acc

def read_dynamic_obstacles(tree):
    dynamicobstacles = tree.find("dynamicobstacles")
    default = dynamicobstacles.find("defaultparameters").attrib
    dos = []
    for obstacle in dynamicobstacles.findall("obstacle"):
        dos.append(
            Obstacle(default, obstacle.attrib, obstacle.findall("section"))
        )
    return dos


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

def render_all(screen, background, dynamic_objects, goal, time):
    bg = background.copy()
    bg.blit(goal[0], goal[1])
    for i in range(len(dynamic_objects)):
        p = dynamic_objects[i]
        if i == 0:
            a_surf, a_rect = render_agent("Agent", p, time)
        else:
            a_surf, a_rect = render_agent("Obstacle", p, time)
        bg.blit(a_surf, a_rect)
    screen.blit(pygame.transform.scale(bg, screen.get_rect().size), (0, 0))
    pygame.display.update()


if __name__ == '__main__':
    tree = ET.parse(sys.argv[1])
    pygame.init()
    map = StaticMap(tree)
    agent_list = Agent.read_agents(tree)
    dynamic_obstacles = read_dynamic_obstacles(tree)
    solved_agent, solution_path = read_solution_path(tree)
    dynamic_objects = [solution_path]
    for obstacle in dynamic_obstacles:
        dynamic_objects.append(obstacle.sections)
    # load and set the logo
    pygame.display.set_caption("minimal program")
    # create a surface on screen that has the size of 240 x 180
    screen = pygame.display.set_mode((pixel_factor*map.width(),pixel_factor*map.height()), pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE)
    screen.fill("grey")
    background, _ = map.render()
    screen.blit(pygame.transform.scale(background, screen.get_rect().size), (0, 0))
    pygame.display.update()
    # define a variable to control the main loop
    running = True
    clock = pygame.time.Clock()
    time_to_done = get_paths_done_time(solution_path)
    start_time = datetime.datetime.now()
    time = 0.0
    # main loop
    while running:
        clock.tick(60)
        # print(update_fps(clock))
        # event handling, gets all event from the event queue
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                print("")
                pygame.display.quit()
                sys.exit()
            elif event.type == pygame.VIDEORESIZE:
                screen = pygame.display.set_mode(event.size, pygame.HWSURFACE|pygame.DOUBLEBUF|pygame.RESIZABLE)
        time = (datetime.datetime.now() - start_time).total_seconds()
        print("\r" + str(time), end = "")
        render_all(screen, background, dynamic_objects, solved_agent.render_goal(),time)
