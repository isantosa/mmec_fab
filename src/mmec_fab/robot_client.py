from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas_rrc
from compas_rrc import MoveToFrame, MoveToJoints, Zone
from compas_fab.backends import RosClient

from mmec_fab import offset_frame
from mmec_fab import ensure_frame

GRIPPER_PIN = "doUnitC1Out1"

# Speed values
ACCEL = 100
ACCEL_RAMP = 100
SPEED_OVERRIDE = 100
TCP_MAX_SPEED = 250

SAFE_JOINT_POSITION = [0, 0, 0, 0, 0, 0]  # six values in degrees

TIMEOUT_SHORT = 10
TIMEOUT_LONG = 30

TOOL = "tool0"
WOBJ = "wobj0"


class RobotClient(compas_rrc.AbbClient):
    """Robot communication client for MMEC

    Subclass of :class:`compas_rrc.AbbClient`.

    Parameters
    ----------
    ros_port : :obj:`int`, optional
        ROS client port for communcation with ABB controller, defaults to 9090.

    Class attributes
    ----------------
    EXTERNAL_AXES_DUMMY : :class:`compas_rrc.ExternalAxes`
        Dummy object used for :class:`compas_rrc.MoveToRobtarget` and
        :class:`MoveToJoints` objects.
    """

    # Define external axes, will not be used but required in move cmds
    EXTERNAL_AXES_DUMMY = compas_rrc.ExternalAxes()

    def __init__(self, ros_port=9090):
        """Sets up a RosClient."""
        super(RobotClient, self).__init__(RosClient(port=9090))

    # __enter__ and __exit__ are called at start and end of with statements
    # example:
    # with RobotClient() as client:
    #     # now __enter__ is executed
    #     client.do_something()
    #     # __exit__ is executed if there's nothing more in the statement
    #
    def __enter__(self):
        self.ros.__enter__()
        return self

    def __exit__(self, *args):
        self.close()
        self.terminate()

    def pre(self, safe_joint_position=[0, 0, 0, 0, 0, 0]):
        self.check_connection_controller()
        # Open gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 0))

        # Set speed and accceleration
        self.send(compas_rrc.SetAcceleration(ACCEL, ACCEL_RAMP))
        self.send(compas_rrc.SetMaxSpeed(SPEED_OVERRIDE, TCP_MAX_SPEED))

        # Set tool and workobject
        self.send(compas_rrc.SetTool(TOOL))
        self.send(compas_rrc.SetWorkObject(WOBJ))

        self.confirm_start()

        self.send_and_wait(
            MoveToJoints(SAFE_JOINT_POSITION, self.EXTERNAL_AXES_DUMMY, 150, 50)
        )

    def post(self, safe_joint_position=[0, 0, 0, 0, 0, 0]):
        self.send_and_wait(
            MoveToJoints(SAFE_JOINT_POSITION, self.EXTERNAL_AXES_DUMMY, 150, 50)
        )

    def pick_place(
        self,
        pick_framelike,
        place_framelike,
        travel_speed=250,
        travel_zone=Zone.Z10,
        precise_speed=50,
        precise_zone=Zone.FINE,
        offset_distance=150,
    ):
        pick_frame = ensure_frame(pick_framelike)
        place_frame = ensure_frame(place_framelike)

        above_pick_frame = offset_frame(pick_frame, -offset_distance)
        above_place_frame = offset_frame(place_frame, -offset_distance)

        # PICK

        # Move to just above pickup frame
        self.send(MoveToFrame(above_pick_frame, travel_speed, travel_zone))

        # Move to pickup frame
        self.send(MoveToFrame(pick_frame, precise_speed, precise_zone))

        # Activate gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 1))

        # Return to just above pickup frame
        self.send(MoveToFrame(above_pick_frame, precise_speed, precise_zone))

        # PLACE

        # Move to just above place frame
        self.send(MoveToFrame(above_place_frame, travel_speed, travel_zone))

        # Move to pickup frame
        self.send(MoveToFrame(pick_frame, precise_speed, precise_zone))

        # Release gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 0))

        # Return to just above pickup frame
        # This command is sent with send_and_wait, to make the client send one
        # pick and place instruction at a time.
        self.send_and_wait(MoveToFrame(above_place_frame, precise_speed, precise_zone))

    def roll(self, framelike_list, offset_distance, speed=50, zone=1):
        frame_list = [ensure_frame(framelike) for framelike in framelike_list]

        for frame in frame_list:
            above_frame = offset_frame(frame, -offset_distance)

            self.send(MoveToFrame(above_frame, speed, zone))
            self.send(MoveToFrame(frame, speed, zone))
            self.send(MoveToFrame(above_frame, speed, zone))

    def confirm_start(self):
        """Stop program and prompt user to press play on pendant to resume."""
        self.send(compas_rrc.PrintText("Press play when ready."))
        self.send(compas_rrc.Stop())
        print("Press start on pendant when ready")

        # After user presses play on pendant execution resumes:
        self.send(compas_rrc.PrintText("Resuming execution."))

    def check_connection_controller(self, timeout=10):
        """Check connection to ABB controller and raises an exception if not connected.

        Parameters
        ----------
        timeout_ping
            Timeout for ping response in seconds.

        Raises
        ------
        :exc:`compas_rrc.TimeoutException`
            If no reply is returned to second ping before timeout.
        """
        try:
            self.send_and_wait(
                compas_rrc.Noop(feedback_level=compas_rrc.FeedbackLevel.DONE),
                timeout=timeout,
            )
        except compas_rrc.TimeoutException:
            raise compas_rrc.TimeoutException(
                "No response from controller. Restart docker container?"
            )
