"""Load frames from JSON and run simple pick and place procedure.

Invoked using `python examples/pick_place_from_json.py examples/pp_frames.json`
"""
from compas import json_load
from compas_rrc import Zone

from mmec_fab import RobotClient


def run_slice_placing(file_path):

    data = json_load(file_path)

    with RobotClient() as client:
        client.pre()

        for pick_slice, safe2, rotated_safe2, place_offset, place_slice in zip(data["pick_slice_frames"], data["safe2_frames"], data["rotated_safe2_frames"], data["place_offset_frames"], data["place_slice_frames"]):
            client.slice_placing(
                pick_slice,
                safe2,
                rotated_safe2,
                place_offset,
                place_slice,
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
        filepath = "02_slice_placing_aa-010.json"
    else:
        print("No input file specified, using example file pp_frames.json")
        filepath = os.path.abspath(os.path.join(__file__, "..", "02_slice_placing_aa-01-010.json"))

    run_slice_placing(filepath)
