#!/usr/bin/env python3
import numpy as np
import xml.etree.ElementTree as ET
import pygame
BLACK = (0, 0, 0)
GREY = (128, 128, 128)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GOLD = (255, 215, 0)


class Agent:
    def __init__(self, defaults, init_config):
        vals = {}
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


def read_map(tree):
    m = tree.find("map")
    grid = m.find("grid")
    map_array = np.zeros((int(grid.attrib["height"]),
                          int(grid.attrib["width"])), dtype=int)
    acc = 0
    for row in grid.findall("row"):
        map_array[acc] = row.text.strip().split(" ")
        acc += 1
    return map_array


def read_agents(tree):
    agents = tree.find("agents")
    agent_list = []
    default = agents.find("defaultparameters").attrib
    for a in agents.findall("agent"):
        ic = a.attrib
        agent_list.append(Agent(default, ic))
    return agent_list


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
    solved_agent = Agent({}, agent.attrib)
    path = agent.find("path")
    solution_path = []
    for s in path.findall("section"):
        solution_path.append(Section(s.attrib))
    return solved_agent, solution_path


if __name__ == '__main__':
    tree = ET.parse('../examples/all_in_one_example_log.xml')
    map_array = read_map(tree)
    agent_list = read_agents(tree)
    dynamic_obstacles = read_dynamic_obstacles(tree)
    solved_agent, solution_path = read_solution_path(tree)
    pygame.init()
    # load and set the logo
    pygame.display.set_caption("minimal program")
    # create a surface on screen that has the size of 240 x 180
    screen = pygame.display.set_mode((10*map_array.shape[1], 10*map_array.shape[0]))
    screen.fill(GREY)
    # define a variable to control the main loop
    running = True

    #draw grid
    for i in range(map_array.shape[0]):
        for j in range(map_array.shape[1]):
            rect = pygame.Rect(10*(j), 10*(i), 10, 10)
            if map_array[i, j] == 1:
                pygame.draw.rect(screen, BLACK, rect, 0)
            else:
                pygame.draw.rect(screen, WHITE, rect, 0)

    for agent in agent_list:
        if agent.id_num != solved_agent.id_num:
            rect = pygame.Rect(10*(agent.start_i), 10*(agent.start_j), 10, 10)
            pygame.draw.rect(screen, RED, rect, 0)
    x = 10*solved_agent.start_i
    y = 10*solved_agent.start_j
    solved_agent_rect = pygame.Rect(x, y, 10, 10)
    pygame.draw.rect(screen, GOLD, solved_agent_rect, 0)
    pygame.display.update()

    clock = pygame.time.Clock()
    i = 0
    tacc = 0.0
    # main loop
    while running:
        # event handling, gets all event from the event queue
        for event in pygame.event.get():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
        p = solution_path[i]
        time = clock.tick(60)
        dt = time/(p.duration*1000)
        tacc += dt
        if tacc > 1.0:
            i += 1
            tacc = 0.0
            continue
        x += 10*dt*(p.i2 - p.i1)
        y += 10*dt*(p.j2 - p.j1)
        print(tacc, x, y)
        solved_agent_rect.update(x, y, 10, 10)
        pygame.draw.rect(screen, GOLD, solved_agent_rect, 0)

        print(solved_agent_rect)
        pygame.display.flip()
