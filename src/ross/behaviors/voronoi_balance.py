import numpy as np

class VoronoiBalanceBehavior:

    def process_scans(self, robots, scans):
        for bot in robots:
        
            # What the robot knows about neighboring robots and Voronoi points
            #data = scans[bot.id]

            # Use messages to "extend" how much data we have 
            messages = bot.get_messages()
            for msg in messages:
                if msg['content']['type'] == 'distance_info':
                    sender_id = msg['from']
                    receiver_dict = next((nbr for nbr in scans[bot.id]['robots'] if nbr['robot_id'] == sender_id), None)
                    if receiver_dict is None:
                        continue
                    else:
                        sender_to_receiver_vector = receiver_dict['vector'] # vector is from our current robot data, so it is sender TO receiver(current robot)

                    pts_in_sender_frame = msg['content']['voronoi_points']
                    pts_in_receiver_frame = []
                    for pt in pts_in_sender_frame:
                        transformed_pt = pt['vector'] + sender_to_receiver_vector
                        pts_in_receiver_frame.append({
                            'point_id': pt['point_id'],
                            'position': pt['position'],
                            'vector': transformed_pt,
                            'distance': np.linalg.norm(transformed_pt)
                        })

                    nbrs_in_sender_frame = msg['content']['neighbors']
                    nbrs_in_receiver_frame = []
                    for nbr in nbrs_in_sender_frame:
                        transformed_nbr = nbr['vector'] + sender_to_receiver_vector
                        nbrs_in_receiver_frame.append({
                            'robot_id': nbr['robot_id'],
                            'vector': transformed_nbr,
                            'distance': np.linalg.norm(transformed_nbr)
                        })

                    # Cross reference the points and neighbors id with our current robot's data
                    for new_pt in pts_in_receiver_frame:
                        match = next((existing_pt for existing_pt in scans[bot.id]['voronoi_points'] if new_pt['point_id'] == existing_pt['point_id']), None)
                        if match:
                            #match['vector'] = np.average([match['vector'], new_pt['vector']], axis=0)
                            #match['distance'] = np.linalg.norm(match['vector'])
                            pass
                        else:
                            scans[bot.id]['voronoi_points'].append(new_pt)

                    for new_nbr in nbrs_in_receiver_frame:
                        match = next((existing_nbr for existing_nbr in scans[bot.id]['robots'] if new_nbr['robot_id'] == existing_nbr['robot_id']), None)
                        if match:
                            #match['vector'] = np.average([match['vector'], new_nbr['vector']], axis=0)
                            #match['distance'] = np.linalg.norm(match['vector'])
                            pass
                        else:
                            scans[bot.id]['robots'].append(new_nbr)
        return scans

    def apply_movement(self, robots, scans):
        for bot in robots:
            
            # What the robot knows about neighboring robots and Voronoi points
            data = scans[bot.id]
            pts  = data['voronoi_points']
            nbrs = data['robots']
            
            # Repulsion from neighbors
            neighbor_repel = np.zeros(2)
            for nbr in nbrs:
                d = nbr['distance']
                if d < 1e-6:
                    continue
                elif d < bot.sensing_radius/2:
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

            # if zero points and zero neighbors, move randomly but bigly (to avoid getting stuck)
            # if len(pts) <= 1 and len(nbrs) == 0:
            #     random_move = np.random.uniform(-1, 1, size=2)
            #     random_move /= np.linalg.norm(random_move)
            #     bot.move_toward(bot.get_position() + random_move * 100)  
        
    
    def send_messages(self, robots, scans):
        for bot in robots:
            # What the robot knows about neighboring robots and Voronoi points
            data = scans[bot.id]
            pts  = data['voronoi_points']
            nbrs = data['robots']

            # Share distances to markers and robots with neighbors
            for nbr in nbrs:
                bot.send_message(
                    recipient_id=nbr['robot_id'],
                    content={
                        'type': 'distance_info',
                        'from': bot.id,
                        'to': nbr['robot_id'],
                        'voronoi_points': pts, 
                        'neighbors': nbrs
                    }
                )


            
