import numpy as np
import open3d as o3d

def update_map(new_points, point_cloud):
    np_points = np.array(new_points)
    point_cloud.points.extend(o3d.utility.Vector3dVector(np_points))
    voxel_size = 0.05
    point_cloud = point_cloud.voxel_down_sample(voxel_size=voxel_size)

    return point_cloud

def is_accessible(target, pcd_array, radius):
    distances = np.linalg.norm(pcd_array - target, axis=1, ord=1)
    inside_circle = pcd_array[distances <= radius]
    if len(inside_circle) == 0:
        return True
    else:
        return False

def expand_points(point, env, render_HZ, resolution, detection_distance, directions, point_cloud, contact_points):
    new_points = set()
    for direction in directions:
        new_point = point + resolution * direction
        env.move_mocap(new_point)
        env.step(nstep=env.HZ//render_HZ)
        env.render()

        is_empty = True
        for sensor_idx in range(360):
            sensor_name = 'rangefinder_' + str(sensor_idx)
            sensor_value = env.get_sensor_value(sensor_name)
            sensor_R = env.get_R_sensor(sensor_name)
            sensor_p = env.get_p_sensor(sensor_name)

            direction_world = sensor_R.dot([0, 0, 1])

            if sensor_value[0] < 7 and sensor_value[0] >= 0: # cutoff : 10.0
                contact_points[sensor_idx] = sensor_p + sensor_value[0] * direction_world
                if sensor_value[0] < detection_distance:
                    is_empty = False
        # Update 2D Map
        point_cloud = update_map(contact_points, point_cloud)
        
        if is_empty: 
            new_points.add(tuple(new_point))

    return new_points, point_cloud
