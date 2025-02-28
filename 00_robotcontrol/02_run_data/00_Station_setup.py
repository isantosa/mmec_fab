"""Load frames from JSON and run simple pick and place procedure.

Invoked using `python examples/pick_place_from_json.py examples/pp_frames.json`
"""
from compas import json_load
from compas_rrc import Zone

from mmec_fab import RobotClient


def run_marking(file_path):

    data = json_load(file_path)

    with RobotClient() as client:
        client.pre()

        for marking, dummy in zip(data["marking_frames"], data["dummy_frames"]):
            client.marking(
                marking,
                dummy,
                travel_speed=250,
                travel_zone=Zone.Z10,
                precise_speed=100,
                precise_zone=Zone.FINE,
                offset_distance=150,
            )
        client.post()


if __name__ == "__main__":
    import os.path
    import sys

    if len(sys.argv) > 1:
        filepath = "00_location_JBtest.json"
    else:
        print("No input file specified, using example file 00_location_JBtest.json")
        filepath = os.path.abspath(os.path.join(__file__, "..", "00_location_JBtest.json"))

    run_marking(filepath)
