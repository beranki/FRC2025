import argparse
import json
import os

def generate_config(robot_name, width, length):
    center_x = width / 2
    center_y = length / 2

    config = {
        "name": f"Robot_{robot_name}",
        "sourceUrl": "",
        "disableSimplification": False,
        "rotations": [],
        "position": [center_x, center_y, 0],
        "cameras": [],
        "components": []
    }

    output_dir = f"Robot_{robot_name}"
    os.makedirs(output_dir, exist_ok=True)
    config_file_path = os.path.join(output_dir, "config.json")

    with open(config_file_path, 'w') as config_file:
        json.dump(config, config_file, indent=4)

    print(f"Config file generated successfully at: {config_file_path}")
    print(f"CAD model: {robot_name}, Width: {width}, Length: {length}")
    return config_file_path

def main():
    parser = argparse.ArgumentParser(description="Generate CAD config for FRC Robot")
    parser.add_argument("name", type=str, help="Robot CAD Name")
    parser.add_argument("width", type=float, help="Width of the CAD model")
    parser.add_argument("length", type=float, help="Length of the CAD model")
    
    args = parser.parse_args()
    generate_config(args.name, args.width, args.length)