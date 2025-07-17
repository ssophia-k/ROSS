import numpy as np
from ross.core.environment import Field
from ross.core.robot import Robot

class Swarm:
    def __init__(self, 
                 field: Field, 
                 spawnable_width_interval: tuple[float, float] | None = None, 
                 spawnable_height_interval: tuple[float, float] | None = None,
                 num_robots: int = 10, 
                 sensing_radius: float = 25.0, 
                 behavior_cls = None):
        self.field = field
        self.robots = []
        self.behavior = behavior_cls()

        # width interval within which bots originally spawn
        if spawnable_width_interval is None:
            w_min, w_max = 0, field.width
        else:
            w_min, w_max = spawnable_width_interval
            # clamp into [0, width]
            w_min = max(0, w_min)
            w_max = min(field.width, w_max)
        if w_min > w_max:
            raise ValueError(
                f"Invalid spawnable_width_interval: "
                f"after clamping to [0, {field.width}] got ({w_min}, {w_max})"
            )
        self.spawnable_width_interval: tuple[int, int] = (w_min, w_max)
        
        # height interval within which bots originally spawn
        if spawnable_height_interval is None:
            h_min, h_max = 0, field.height
        else:
            h_min, h_max = spawnable_height_interval
            # clamp into [0, height]
            h_min = max(0, h_min)
            h_max = min(field.height, h_max)
        if h_min > h_max:
            raise ValueError(
                f"Invalid spawnable_height_interval: "
                f"after clamping to [0, {field.height}] got ({h_min}, {h_max})"
            )
        self.spawnable_height_interval: tuple[int, int] = (h_min, h_max)

        for i in range(num_robots):
            x = np.random.uniform(w_min, w_max)
            y = np.random.uniform(h_min, h_max)
            bot = Robot(x, y, robot_id=i, field=field, sensing_radius=sensing_radius)
            bot.set_swarm(self)
            self.robots.append(bot)


    def step(self):
        scans = {bot.id: bot.scan() for bot in self.robots}
        self.behavior.send_messages(self.robots, scans)
        self.behavior.apply_movement(self.robots, scans)


    def deliver_message(self, sender_id: int, recipient_id: int, content: dict) -> None:
        """Deliver a message from one robot to another."""
        for bot in self.robots:
            if bot.id == recipient_id:
                bot.receive_message(sender_id, content)
                break
