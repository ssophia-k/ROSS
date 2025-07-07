import numpy as np

class VoronoiBalanceBehavior:
    def apply(self, robots, scans):
        for bot in robots:
            messages = bot.get_messages()
            for msg in messages:
                pass # TODO: Process message content

            data = scans[bot.id]
            pts  = data['voronoi_points']
            nbrs = data['robots']

            # if len(pts) < 2:
            #     continue
            # commented out to allow for neighbor repulsion even with one point

            # Repulsion from neighbors
            neighbor_repel = np.zeros(2)
            for nbr in nbrs:
                d = nbr['distance']
                if d < 1e-6:
                    continue
                dirn = nbr['vector'] / d
                neighbor_repel -= dirn * (1.0 / d**2)
            neighbor_repel *= 5 # tune this gain to spread more or less

            # Voronoi point attraction/repulsion
            sorted_pts = sorted(pts, key=lambda d: d['distance'])

            # only look at the closest 2 points
            point_drive = np.zeros(2)
            if len(sorted_pts) >= 2:
                sorted_pts = sorted_pts[:2]
                u_closest = sorted_pts[0]['vector'] / sorted_pts[0]['distance']
                u_farthest = sorted_pts[-1]['vector'] / sorted_pts[-1]['distance']
                point_drive = (u_farthest - u_closest)
            norm = np.linalg.norm(point_drive)
            if norm > 1e-6:
                point_drive /= norm

            move_vec = point_drive + neighbor_repel
            if np.linalg.norm(move_vec) > 1e-2:
                bot.move_toward(bot.get_position() + move_vec/np.linalg.norm(move_vec))

            # Share distances to markers and robots with neighbors
            for nbr in nbrs:
                bot.send_message(
                    recipient_id=nbr['robot_id'],
                    content={
                        'type': 'distance_info',
                        'from': bot.id,
                        'to': nbr['robot_id'],
                        'distance_to_marker': [p['distance'] for p in pts],
                        'distance_to_robots': [n['distance'] for n in nbrs]
                    }
                )
