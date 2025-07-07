import numpy as np
from ross.core.environment import Field
from ross.core.robot import Robot

class Swarm:
    def __init__(self, field: Field, num_robots: int, sensing_radius: float, behavior_cls):
        self.field = field
        self.robots = []
        self.behavior = behavior_cls()

        for i in range(num_robots):
            x, y = np.random.rand() * field.width, np.random.rand() * field.height
            bot = Robot(x, y, robot_id=i, field=field, sensing_radius=sensing_radius)
            bot.set_swarm(self)
            self.robots.append(bot)

    def step(self):
        scans = {bot.id: bot.scan() for bot in self.robots}
        self.behavior.apply(self.robots, scans)

    def deliver_message(self, sender_id: int, recipient_id: int, content: dict) -> None:
        """Deliver a message from one robot to another."""
        for bot in self.robots:
            if bot.id == recipient_id:
                bot.receive_message(sender_id, content)
                break
