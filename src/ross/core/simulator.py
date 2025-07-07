import numpy as np

class Simulator:
    def __init__(self, field, swarm, steps=1000):
        self.field = field
        self.swarm = swarm
        self.steps = steps
        self.robot_paths = [[] for _ in range(len(swarm.robots))]

        # Record initial positions
        for i, bot in enumerate(self.swarm.robots):
            self.robot_paths[i].append(bot.get_position())

    def step(self):
        self.swarm.step()
        for i, bot in enumerate(self.swarm.robots):
            self.robot_paths[i].append(bot.get_position())

    def run(self):
        for _ in range(self.steps):
            self.step()

    def get_paths(self):
        return self.robot_paths
