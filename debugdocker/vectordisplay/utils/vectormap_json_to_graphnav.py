from argparse import ArgumentParser
from pathlib import Path
from typing import List, Dict
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

    nodes = []
    edges = []

    for vector in vectors:
        p0 = vector['p0']
        p1 = vector['p1']

        node1 = create_point(len(nodes), p0)
        node2 = create_point(len(nodes) + 1, p1)

        edge = create_edge(node1['id'], node2['id'])

        nodes.append(node1)
        nodes.append(node2)
        edges.append(edge)

    with output_file.open('w') as f:
        json.dump({'edges': edges, 'nodes': nodes}, f, separators=(',', ':'), indent=2)
    print(f"Graph Nav written out to: {output_file}")


def create_point(id: int, loc: Dict[str, any]) -> Dict[str, any]:
    return {
        "id": id,
        "loc": {
            "x": loc.get('x'),
            "y": loc.get("y")
        }
    }


def create_edge(p0id: int, p1id: int) -> Dict[str, any]:
    return {
        'has_automated_door': False,
        'has_door': False,
        'has_elevator': False,
        'has_stairs': False,
        'max_clearance': 10.0,
        'max_speed': 50.0,
        's0_id': p0id,
        's1_id': p1id
    }


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
