import rospy
from nav_msgs.srv import GetMap
import os
import yaml

def save_map():
    rospy.init_node('map_saver')
    rospy.wait_for_service('/static_map')
    map_service = rospy.ServiceProxy('/static_map', GetMap)

    try:
        map_resp = map_service()
        map_data = map_resp.map

        output_dir = rospy.get_param('~output_dir', '/tmp')
        filename = rospy.get_param('~map_filename', 'map')
        
        # Ensure the output directory exists
        os.makedirs(output_dir, exist_ok=True)

        # Save PGM file
        pgm_path = os.path.join(output_dir, f"{filename}.pgm")
        with open(pgm_path, 'wb') as pgm_file:
            pgm_file.write(f"P5\n{map_data.info.width} {map_data.info.height}\n255\n".encode())
            pgm_file.write(bytes(map_data.data))

        # Create YAML metadata
        yaml_data = {
            'image': f"{filename}.pgm",
            'resolution': map_data.info.resolution,
            'origin': [map_data.info.origin.position.x, map_data.info.origin.position.y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }

        # Save YAML file
        yaml_path = os.path.join(output_dir, f"{filename}.yaml")
        with open(yaml_path, 'w') as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False)

        rospy.loginfo(f"Map saved at {yaml_path} and {pgm_path}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    save_map()
