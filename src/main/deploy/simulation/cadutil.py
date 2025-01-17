import argparse
import json
import os
from typing import Dict


def generate_config(robot_name: str, width: float, length: float) -> str:
    center_x: float = width / 2
    center_y: float = length / 2
    print(f"Center position of the CAD model (x, y): ({center_x}, {center_y})")

    config: Dict[str, any] = {
        "name": f"Robot_{robot_name}",
        "sourceUrl": "",
        "disableSimplification": False,
        "rotations": [

    {
      "axis": "x",
      "degrees": 90
    },
    {
      "axis": "z",
      "degrees": 90
    }

        ],
        "position": [center_x, center_y, 0],
        "cameras": [],
        "components": []
    }

    output_dir: str = f"Robot_{robot_name}"
    os.makedirs(output_dir, exist_ok=True)

    config_file_path: str = os.path.join(output_dir, "config.json")

    if os.path.exists(config_file_path):
        user_response: str = input(
            f"The file '{config_file_path}' already exists. Do you want to replace it? (y/n): ").strip().lower()
        if user_response != 'y':
            print("Operation aborted by the user.")
            return config_file_path

    with open(config_file_path, 'w') as config_file:
        json.dump(config, config_file, indent=4)

    print(f"Config file generated successfully at: {config_file_path}")
    print(f"CAD model: {robot_name}, Width(meters): {width}, Length(meters): {length}")
    return config_file_path


def main() -> None:
    parser: argparse.ArgumentParser = argparse.ArgumentParser(description="Generate CAD config for FRC Robot")
    parser.add_argument("name", type=str, help="Robot CAD Name")
    parser.add_argument("width", type=float, help="Width of the CAD model")
    parser.add_argument("length", type=float, help="Length of the CAD model")

    args: argparse.Namespace = parser.parse_args()
    generate_config(args.name, args.width, args.length)


if __name__ == "__main__":
    main()

