from argparse import ArgumentParser
from pathlib import Path
from typing import List
import json

UTILS_FOLDER = Path(__file__).parent
VECTOR_DISPLAY_FOLDER = UTILS_FOLDER.parent
MAPS_FOLDER = VECTOR_DISPLAY_FOLDER / 'maps'


def vectormap_json_to_pedsim(
    input_file: Path,
    output_file: Path
):
    """
    Creates a PedSim configuration file converting the information given from a vectormap json file.

    :param input_file: The json input vectormap location
    :param output_file: The place to write out the xml file to.
    """

    xml_obstacles: List[str] = []
    vectors = json.load(input_file.open('r'))

    for vector in vectors:
        p0 = vector['p0']
        p1 = vector['p1']

        obstacle = f'<obstacle x1="{p0["x"]}" y1="{p0["y"]}" x2="{p1["x"]}" y2="{p1["y"]}"/>'
        xml_obstacles.append(obstacle)

    xml_file_content = create_scene_xml(xml_obstacles)
    with output_file.open('w') as f:
        f.write(xml_file_content)
    print(f"XML Scenario written out to: {output_file}")


def create_scene_xml(
        obstacles: List[str]
) -> str:
    """
    Helper function for creating a PedSim XML configuration file.

    :param obstacles: List of strings that are of the standard XML pedsim obstacle format, i.e.:
            <obstacle x1="-8.4992" y1="7.02464" x2="-6.98368" y2="7.02464"/>
    :returns: The content of the PedSim config file as a string that can be written out/stored/etc.
    """

    obstacle_join = "\n\t"

    return f"""
<?xml version="1.0" encoding="UTF-8"?>
<scenario>
    <!--Obstacles-->
    {obstacle_join.join(obstacles)}    
    <!--Way Points-->
    
    <!-- This Robot Goal Doesn't Matter, but is Required -->
    <waypoint id="robot_goal" x="22" y="27" r="2"/>
    <waypoint id="robot_start" x="-1.45" y="10.35" r="2"/>
    <agent x="-1.45" y="10.35" n="1" dx="0" dy="0" type="2">
        <addwaypoint id="robot_start"/>
        <addwaypoint id="robot_goal"/>
    </agent>
    
    <!-- Agents -->
</scenario>
"""


if __name__ == "__main__":
    parser = ArgumentParser()

    parser.add_argument('--input_file', '-i', type=str, required=True,
                        help='The file that contains the textual vector map to convert.')
    parser.add_argument('--output_file', '-o', type=str, required=True,
                        help='Where to save the json output vectormap.')


    args = parser.parse_args()

    input_file: Path = Path(args.input_file)
    output_file: Path = Path(args.output_file)

    vectormap_json_to_pedsim(
        input_file,
        output_file
    )
