from argparse import ArgumentParser
from pathlib import Path
import json

UTILS_FOLDER = Path(__file__).parent
VECTOR_DISPLAY_FOLDER = UTILS_FOLDER.parent
MAPS_FOLDER = VECTOR_DISPLAY_FOLDER / 'maps'


def vectormap_txt_to_json(
        vectormap_text_file: Path,
        vectormap_json_output_file: Path,
):
    """
    Helper function for turning the txt formatted vectormap files into json vector maps (TODO - is this needed?)

    :param vectormap_text_file: The path object to a x.vectormap.txt file.
    :param vectormap_json_output_file: Where you want to store the output vectormap.json file.
    """

    assert vectormap_text_file.name.endswith('.vectormap.txt'), 'please specify a correct {x}.vectormap.txt file'
    assert vectormap_json_output_file.name.endswith('.vectormap.json'), \
        'please specify a correct {x}.vectormap.json file'

    vectors = []

    with vectormap_text_file.open('r') as r:
        lines = r.readlines()
        for line in lines:
            values = line.strip().split(" ")
            p0x, p0y, p1x, p1y = values

            vectors.append({'p0': {'x': p0x, 'y': p0y}, 'p1': {'x': p1x, 'y': p1y}})

    with vectormap_json_output_file.open('w') as w:
        json.dump(vectors, w)


if __name__ == "__main__":
    parser = ArgumentParser()

    parser.add_argument('--input_file', '-i', type=str, required=True,
                        help='The file that contains the textual vector map to convert.')
    parser.add_argument('--output_file', '-o', type=str, required=True,
                        help='Where to save the json output vectormap.')


    args = parser.parse_args()

    input_file: Path = Path(args.input_file)
    output_file: Path = Path(args.output_file)

    vectormap_txt_to_json(
        input_file,
        output_file
    )
