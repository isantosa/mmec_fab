"""Load frames from JSON and run simple pick and place procedure.

Invoked using `python examples/pick_place_from_json.py examples/pp_frames.json`
"""
from compas import json_load
from compas_rrc import Zone

from mmec_fab import RobotClient


def run_pick_place(file_path):

    data = json_load(file_path)

    with RobotClient() as client:
        client.pre()

        pick_frames = data["pick_frames"]
        place_frames = data["place_frames"]
        travel_frames = data["travel_frames"]

        # non zip example
        """
        for n in range(len(pick_frames):
            client.pick_place(
                pick_frames[n],
                place_frames[n],
                data["travel_frame"],
                travel_speed=250,
                travel_zone=Zone.Z10,
                precise_speed=50,
                precise_zone=Zone.FINE,
                offset_distance=150,
            )
        """

        # zip example
        for pick, place in zip(pick_frames, place_frames):
            client.pick_place(
                pick,
                place,
                travel_frames,
                travel_speed=250,
                travel_zone=Zone.Z10,
                precise_speed=50,
                precise_zone=Zone.FINE,
                offset_distance=150,
            )
        client.post()


if __name__ == "__main__":
    import os.path
    import sys

    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        print("No input file specified, using example file pp_frames.json")
        filepath = os.path.abspath(os.path.join(__file__, "..", "pp_frames.json"))

    run_pick_place(filepath)
