from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import compas_rrc
from compas_rrc import MoveToFrame, MoveToJoints, Zone, Motion
from compas_fab.backends import RosClient

from mmec_fab import offset_frame
from mmec_fab import ensure_frame

GRIPPER_PIN = "doUnitC1Out1"

# Speed values
ACCEL = 100  # %
ACCEL_RAMP = 100  # %
SPEED_OVERRIDE = 100  # %
TCP_MAX_SPEED = 250  # mm/s

SAFE_JOINT_POSITION = [0, 0, 0, 0, 90, 0]  # six values in degrees

TIMEOUT_SHORT = 10
TIMEOUT_LONG = 30

TOOL = "tool0"
TOOL_CN = "t_A057_CalibrationNeedle"
TOOL_MMW = "t_A057_MMWTool03"
WOBJ = "wobj0"
WOBJ_SL = "ob_A057_WobjSliceST"
WOBJ_CT = "ob_A057_WobjCutST"
WOBJ_LT = "ob_A057_WobjLatticeST"


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
        super(RobotClient, self).__init__(RosClient(port=9090), namespace="/")

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
        self.ros.close()
        self.ros.terminate()

    def pre(self, safe_joint_position=[0, 0, 0, 0, 90, 0]):
        self.check_connection_controller()
        # Open gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, False))

        # Set speed and accceleration
        self.send(compas_rrc.SetAcceleration(ACCEL, ACCEL_RAMP))
        self.send(compas_rrc.SetMaxSpeed(SPEED_OVERRIDE, TCP_MAX_SPEED))

        # Set tool and workobject
        # self.send(compas_rrc.SetTool(TOOL))
        self.send(compas_rrc.SetTool(TOOL_MMW))
        self.send(compas_rrc.SetWorkObject(WOBJ))

        self.confirm_start()

        self.send_and_wait(
            MoveToJoints(SAFE_JOINT_POSITION, self.EXTERNAL_AXES_DUMMY, 150, 50)
        )
        self.send(compas_rrc.PrintText("Start Production"))


    def post(self, safe_joint_position=[0, 0, 0, 0, 90, 0]):
        self.send_and_wait(
            MoveToJoints(SAFE_JOINT_POSITION, self.EXTERNAL_AXES_DUMMY, 150, 50)
        )
        self.send(compas_rrc.PrintText("Finish Production"))


    def pre_rolling(self, safe_joint_position=[90, 0, 0, 0, 90, 0]):
        self.check_connection_controller()
        # Open gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, False))

        # Set speed and accceleration
        self.send(compas_rrc.SetAcceleration(ACCEL, ACCEL_RAMP))
        self.send(compas_rrc.SetMaxSpeed(SPEED_OVERRIDE, TCP_MAX_SPEED))

        # Set tool and workobject
        # self.send(compas_rrc.SetTool(TOOL))
        self.send(compas_rrc.SetTool(TOOL_MMW))
        self.send(compas_rrc.SetWorkObject(WOBJ))

        self.confirm_start()

        self.send_and_wait(
            MoveToJoints(SAFE_JOINT_POSITION, self.EXTERNAL_AXES_DUMMY, 150, 50)
        )
        self.send(compas_rrc.PrintText("Start Production"))


    def post_rolling(self, safe_joint_position=[90, 0, 0, 0, 90, 0]):
        self.send_and_wait(
            MoveToJoints(SAFE_JOINT_POSITION, self.EXTERNAL_AXES_DUMMY, 150, 50)
        )
        self.send(compas_rrc.PrintText("Finish Production"))



    def pick_place(
        self,
        pick_framelike,
        place_framelike,
        travel_speed=250,
        travel_zone=Zone.Z10,
        precise_speed=50,
        precise_zone=Zone.FINE,
        offset_distance=150,
        motion_type_travel=Motion.JOINT,
        motion_type_precise=Motion.LINEAR,
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
        self.send(MoveToFrame(above_pick_frame, precise_speed, precise_zone,motion_type=motion_type_precise))

        # PLACE

        # Move to just above place frame
        self.send(MoveToFrame(above_place_frame, travel_speed, travel_zone))

        # Move to pickup frame
        self.send(MoveToFrame(place_frame, precise_speed, precise_zone))

        # Release gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 0))

        # Move to just above place frame
        self.send(MoveToFrame(above_place_frame, travel_speed, travel_zone))


    def base_making(
        self,
        pick_framelike,
        measure_framelike,
        safeb1_framelike,
        safeb2_framelike,
        place_framelike,
        # travel_speed=250,
        travel_speed=1000,
        travel_zone=Zone.Z10,
        precise_speed=50,
        precise_zone=Zone.FINE,
        offset_distance=150,
        motion_type_travel=Motion.JOINT,
        motion_type_precise=Motion.LINEAR,
    ):
        pick_frame = ensure_frame(pick_framelike)
        measure_frame = ensure_frame(measure_framelike)
        safeb1_frame = ensure_frame(safeb1_framelike)
        safeb2_frame = ensure_frame(safeb2_framelike)
        place_frame = ensure_frame(place_framelike)

        above_pick_frame = offset_frame(pick_frame, -offset_distance)
        above_measure_frame = offset_frame(measure_frame, -offset_distance)
        above_place_frame = offset_frame(place_frame, -offset_distance)


        #### MOVE TO SAFE POINT

        # Set Workobject to World Object 0
        self.send(compas_rrc.SetWorkObject(WOBJ))

        # Safepoint b1 to start
        self.send(MoveToFrame(safeb1_frame, travel_speed, travel_zone, motion_type=motion_type_precise))


        #### Move to CUTTING STATION

        # Set Workobject to Cutting Station
        self.send(compas_rrc.SetWorkObject(WOBJ_CT))

        # Move to just above pickup frame
        self.send_and_wait(MoveToFrame(above_pick_frame, travel_speed, travel_zone))

        # Move to pickup frame
        self.send(MoveToFrame(pick_frame, precise_speed, precise_zone))

        # Activate gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 1))

        # Slide to measure wood before cutting
        self.send(MoveToFrame(measure_frame, precise_speed, precise_zone, motion_type=motion_type_precise))

        # Stop to allow human to Cut the Wood
        self.stop_to_cut()

        # Move to just above measure frame
        self.send(MoveToFrame(above_measure_frame, travel_speed, travel_zone))


        #### MOVE TO SAFE POINTS

        # Set Workobject to World Object 0
        self.send(compas_rrc.SetWorkObject(WOBJ))

         # First Safe_b1 point
        self.send(MoveToFrame(safeb1_frame, travel_speed, travel_zone))

         # Second Safe_b2 point
        self.send_and_wait(MoveToFrame(safeb2_frame, travel_speed, travel_zone))


        #### Move TO LATTICE MAKING STATION

        # Set Workobject to lattice making slice
        self.send(compas_rrc.SetWorkObject(WOBJ_LT))

        # Move to just above place frame
        self.send(MoveToFrame(above_place_frame, travel_speed, travel_zone))

        # Move to place frame
        self.send(MoveToFrame(place_frame, precise_speed, precise_zone))

        # Stop to allow human to nail the Wood
        self.stop_to_nail()

        # Release gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 0))

        # Move to just above place frame
        self.send_and_wait(MoveToFrame(above_place_frame, travel_speed, travel_zone))
        # self.send_and_wait(MoveToFrame(safe_frame, travel_speed, travel_zone))
        

        # RETURN TO SAVE POINT
        # This command is sent with send_and_wait, to make the client send one
        # pick and place instruction at a time.
        #self.send_and_wait(MoveToFrame(safe_frame, travel_speed, travel_zone))


    def slice_making(
        self,
        pick_framelike,
        measure_framelike,
        safe_framelike,
        place_framelike,
        # travel_speed=250,
        travel_speed=1000,
        travel_zone=Zone.Z10,
        precise_speed=50,
        precise_zone=Zone.FINE,
        offset_distance=150,
        motion_type_travel=Motion.JOINT,
        motion_type_precise=Motion.LINEAR,
    ):
        pick_frame = ensure_frame(pick_framelike)
        measure_frame = ensure_frame(measure_framelike)
        safe_frame = ensure_frame(safe_framelike)
        place_frame = ensure_frame(place_framelike)

        above_pick_frame = offset_frame(pick_frame, -offset_distance)
        above_measure_frame = offset_frame(measure_frame, -offset_distance)
        above_place_frame = offset_frame(place_frame, -offset_distance)


        #### MOVE TO SAFE POINT

        # Set Workobject to World Object 0
        self.send(compas_rrc.SetWorkObject(WOBJ))

        # Safepoint
        self.send(MoveToFrame(safe_frame, travel_speed, travel_zone,motion_type=motion_type_precise))

        #### MOVEMENT AT THE CUTTING STATION

        # Set Workobject to Cutting Station
        self.send(compas_rrc.SetWorkObject(WOBJ_CT))

        # Move to just above pickup frame
        self.send(MoveToFrame(above_pick_frame, travel_speed, travel_zone))

        # Move to pickup frame
        self.send(MoveToFrame(pick_frame, precise_speed, precise_zone))

        # Activate gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 1))

        # Slide to measure wood before cutting
        self.send_and_wait(MoveToFrame(measure_frame, precise_speed, precise_zone, motion_type=motion_type_precise))

        # Stop to allow human to Cut the Wood
        self.stop_to_cut()

        # Move to just above measure frame
        self.send(MoveToFrame(above_measure_frame, travel_speed, travel_zone))

        #### MOVE TO SAFE POINT

        # Set Workobject to World Object 0
        self.send(compas_rrc.SetWorkObject(WOBJ))

         # Safepoint
        self.send(MoveToFrame(safe_frame, travel_speed, travel_zone))


        #### MOVEMENT AT THE SLICE MAKING STATION

        # Set Workobject to Slice Making Station
        self.send(compas_rrc.SetWorkObject(WOBJ_SL))

        # Move to just above place frame
        self.send(MoveToFrame(above_place_frame, travel_speed, travel_zone))

        # Move to place frame
        self.send(MoveToFrame(place_frame, precise_speed, precise_zone))

        # Stop to allow human to nail the Wood
        self.stop_to_nail()

        # Release gripper
        self.send_and_wait(compas_rrc.SetDigital(GRIPPER_PIN, 0))

        # Move to just above place frame
        self.send_and_wait(MoveToFrame(above_place_frame, travel_speed, travel_zone))
        # self.send_and_wait(MoveToFrame(safe_frame, travel_speed, travel_zone))
        

        # RETURN TO SAVE POINT
        # This command is sent with send_and_wait, to make the client send one
        # pick and place instruction at a time.
        # self.send_and_wait(MoveToFrame(safe_frame, travel_speed, travel_zone))


    ####

    def slice_placing(
        self,
        pick_slice_framelike,
        safe2_framelike,
        rotated_safe2_framelike,
        place_offset_framelike,
        place_slice_framelike,
        # travel_speed=250,
        travel_speed=1000,
        travel_zone=Zone.Z10,
        precise_speed=50,
        precise_zone=Zone.FINE,
        offset_distance=150,
        motion_type_travel=Motion.JOINT,
        motion_type_precise=Motion.LINEAR,
    ):
        pick_slice_frame = ensure_frame(pick_slice_framelike)
        safe2_frame = ensure_frame(safe2_framelike)
        rotated_safe2_frame = ensure_frame(rotated_safe2_framelike)
        place_offset_frame = ensure_frame(place_offset_framelike)
        place_slice_frame = ensure_frame(place_slice_framelike)

        above_pick_slice_frame = offset_frame(pick_slice_frame, -offset_distance)
        offset_place_slice_frame = offset_frame(place_slice_frame, -offset_distance)

        #### MOVEMENT AT THE SLICE MAKING STATION

        # Set Workobject to Slice Making Station
        self.send(compas_rrc.SetWorkObject(WOBJ_SL))
        
        #### MOVE TO SAFE2 POINT
        self.send_and_wait(MoveToFrame(safe2_frame, travel_speed, travel_zone))

        # Move to just above pick_slice frame
        self.send(MoveToFrame(above_pick_slice_frame, travel_speed, travel_zone))

        # Move to pick_slice frame
        self.send(MoveToFrame(pick_slice_frame, precise_speed, precise_zone))

        # Activate gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 1))

        # Move to just above pick_slice frame
        self.send(MoveToFrame(above_pick_slice_frame, travel_speed, travel_zone))

        #### MOVE TO SAFE2 POINT
        self.send(MoveToFrame(safe2_frame, travel_speed, travel_zone))

        # Rotate plane at safe2 point
        self.send(MoveToFrame(rotated_safe2_frame, precise_speed, precise_zone, motion_type=motion_type_precise))

        #### MOVEMENT AT THE LATTICE MAKING STATION

        # Set Workobject to Lattice Station
        self.send(compas_rrc.SetWorkObject(WOBJ_LT))

        # Move to place_offset frame
        self.send_and_wait(MoveToFrame(place_offset_frame, travel_speed, travel_zone, motion_type=motion_type_precise))

        # Move to place_slice frame
        self.send(MoveToFrame(place_slice_frame, precise_speed, precise_zone, motion_type=motion_type_precise))

        # Stop to allow human to nail the Wood
        self.stop_to_nail()

        # Release gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 0))

        # Move to offset place_slice frame
        self.send(MoveToFrame(offset_place_slice_frame, precise_speed, precise_zone, motion_type=motion_type_precise))

        # Set Workobject to Slice Making Station
        self.send(compas_rrc.SetWorkObject(WOBJ_SL))

        # move to rotated_safe2 plane
        self.send_and_wait(MoveToFrame(rotated_safe2_frame, travel_speed, travel_zone, motion_type=motion_type_precise))


    ####

    def cap_making(
        self,
        pick_framelike,
        measure_framelike,
        safe_framelike,
        place_framelike,
        # travel_speed=250,
        travel_speed=1000,
        travel_zone=Zone.Z10,
        precise_speed=50,
        precise_zone=Zone.FINE,
        offset_distance=150,
        motion_type_travel=Motion.JOINT,
        motion_type_precise=Motion.LINEAR,
    ):
        pick_frame = ensure_frame(pick_framelike)
        measure_frame = ensure_frame(measure_framelike)
        safe_frame = ensure_frame(safe_framelike)
        place_frame = ensure_frame(place_framelike)

        above_pick_frame = offset_frame(pick_frame, -offset_distance)
        above_measure_frame = offset_frame(measure_frame, -offset_distance)
        above_place_frame = offset_frame(place_frame, -offset_distance)

        #### MOVE TO SAFE POINT

        # Set Workobject to World Object 0
        self.send(compas_rrc.SetWorkObject(WOBJ))

        # Move to Safepoint 
        self.send(MoveToFrame(safe_frame, travel_speed, travel_zone, motion_type=motion_type_precise))


        #### Move to CUTTING STATION

        # Set Workobject to Cutting Station
        self.send(compas_rrc.SetWorkObject(WOBJ_CT))

        # Move to just above pickup frame
        self.send_and_wait(MoveToFrame(above_pick_frame, travel_speed, travel_zone))

        # Move to pickup frame
        self.send(MoveToFrame(pick_frame, precise_speed, precise_zone))

        # Activate gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 1))

        # Slide to measure wood before cutting
        self.send(MoveToFrame(measure_frame, precise_speed, precise_zone, motion_type=motion_type_precise))

        # Stop to allow human to Cut the Wood
        self.stop_to_cut()

        # Move to just above measure frame
        self.send(MoveToFrame(above_measure_frame, travel_speed, travel_zone))


        #### MOVE TO SAFE POINT

        # Set Workobject to World Object 0
        self.send(compas_rrc.SetWorkObject(WOBJ))

         # Safe point
        self.send(MoveToFrame(safe_frame, travel_speed, travel_zone))


        #### Move TO SLICE MAKING STATION

        # Set Workobject to slice making slice
        self.send(compas_rrc.SetWorkObject(WOBJ_SL))

        # Move to just above place frame
        self.send(MoveToFrame(above_place_frame, travel_speed, travel_zone))

        # Move to place frame
        self.send(MoveToFrame(place_frame, precise_speed, precise_zone))

        # Stop to allow human to nail the Wood
        self.stop_to_nail()

        # Release gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 0))

        # Move to just above place frame
        self.send_and_wait(MoveToFrame(above_place_frame, travel_speed, travel_zone))
        # self.send_and_wait(MoveToFrame(safe_frame, travel_speed, travel_zone))
        

        # RETURN TO SAVE POINT
        # This command is sent with send_and_wait, to make the client send one
        # pick and place instruction at a time.
        #self.send_and_wait(MoveToFrame(safe_frame, travel_speed, travel_zone))

    ####

    def point_go(
        self,
        pick_framelike,
        place_framelike,
        travel_speed=250,
        travel_zone=Zone.Z10,
        precise_speed=50,
        precise_zone=Zone.FINE,
        offset_distance=150,
        motion_type_travel=Motion.JOINT,
        motion_type_precise=Motion.LINEAR,
    ):
        pick_frame = ensure_frame(pick_framelike)

        # PICK

        # Move to pickup frame
        self.send_and_wait(MoveToFrame(pick_frame, precise_speed, precise_zone, motion_type=motion_type_precise))

        # Stop to measure
        self.stop_to_measure()

    def marking(
        self,
        marking_framelike,
        dummy_framelike,
        travel_speed=250,
        travel_zone=Zone.Z10,
        precise_speed=50,
        precise_zone=Zone.FINE,
        offset_distance=150,
        motion_type_travel=Motion.JOINT,
        motion_type_precise=Motion.LINEAR,
    ):
        marking_frame = ensure_frame(marking_framelike)

        # GO TO LOCATION POINT

        # Set Workobject
        self.send(compas_rrc.SetWorkObject(WOBJ_SL))
        # self.send(compas_rrc.SetWorkObject(WOBJ_CT))
        # self.send(compas_rrc.SetWorkObject(WOBJ_LT))

        # Move to frame
        self.send_and_wait(MoveToFrame(marking_frame, precise_speed, precise_zone,motion_type_precise))

        # Stop to measure
        self.stop_to_measure()

    ####

    def rolling(
        self,
        rolling_framelike,
        saferight_framelike,
        travel_speed=250,
        travel_zone=Zone.Z10,
        precise_speed=50,
        precise_zone=Zone.FINE,
        offset_distance=4,
        motion_type_travel=Motion.JOINT,
        motion_type_precise=Motion.LINEAR,
    ):
        rolling_frame = ensure_frame(rolling_framelike)
        offset_rolling_frame = offset_frame(rolling_frame, -offset_distance)
        # saferight_frame = ensure_frame(rolling_framelike) ----- (need to do loop in loop)

    

    
        #### MOVEMENT AT THE SLICE MAKING STATION

        # Set Workobject to Lattice Station
        self.send(compas_rrc.SetWorkObject(WOBJ_LT))

        # Open gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 0))

        # Stop to measure
        self.stop_to_nail()
        
        # Move to frame
        self.send_and_wait(MoveToFrame(offset_rolling_frame, precise_speed, precise_zone,motion_type_precise))

        # Move to frame
        self.send_and_wait(MoveToFrame(rolling_frame, precise_speed, precise_zone,motion_type_precise))

        # Close gripper
        self.send(compas_rrc.SetDigital(GRIPPER_PIN, 1))

        # Move to frame
        self.send_and_wait(MoveToFrame(rolling_frame, precise_speed, precise_zone,motion_type_precise))

        # Stop to measure
        self.stop_to_nail()

    ####

    # def roll(
    # self, framelike_list, 
    # offset_distance, 
    # speed=50, 
    # zone=1
    # ):
    #     frame_list = [ensure_frame(framelike) for framelike in framelike_list]

    #     for frame in frame_list:
    #         above_frame = offset_frame(frame, -offset_distance)

    #         self.send(MoveToFrame(above_frame, speed, zone))
    #         self.send(MoveToFrame(frame, speed, zone))
    #         self.send(MoveToFrame(above_frame, speed, zone))



    # def roll(self, framelike_list, offset_distance, speed=50, zone=1):
    #     frame_list = [ensure_frame(framelike) for framelike in framelike_list]

    #     for frame in frame_list:
    #         above_frame = offset_frame(frame, -offset_distance)

    #         self.send(MoveToFrame(above_frame, speed, zone))
    #         self.send(MoveToFrame(frame, speed, zone))
    #         self.send(MoveToFrame(above_frame, speed, zone))

    def confirm_start(self):
        """Stop program and prompt user to press play on pendant to resume."""
        self.send(compas_rrc.PrintText("Press play To start the Program."))
        self.send(compas_rrc.Stop())
        print("Press start on pendant when ready")

        # After user presses play on pendant execution resumes:
        self.send(compas_rrc.PrintText("Resuming execution."))


    def stop_to_cut(self):
        """Stop program and prompt user to press play on pendant to resume."""
        self.send(compas_rrc.PrintText("stop to Cut, press play When Finish."))
        self.send(compas_rrc.Stop())
        print("stop to Cut, press play on pendant to continue")

        # After user presses play on pendant execution resumes:
        self.send(compas_rrc.PrintText("continue to place and nail process."))

    def stop_to_nail(self):
        """Stop program and prompt user to press play on pendant to resume."""
        self.send(compas_rrc.PrintText("stop to Nail, press play when Finish."))
        self.send(compas_rrc.Stop())
        print("stop to Nail, press play on pendant to continue")

        # After user presses play on pendant execution resumes:
        self.send(compas_rrc.PrintText("continue to pick and cut process."))

    def stop_to_measure(self):
        """Stop program and prompt user to press play on pendant to resume."""
        self.send(compas_rrc.PrintText("stop to measure, press play when Finish."))
        self.send(compas_rrc.Stop())
        print("stop to measure, press play on pendant to continue")

        # After user presses play on pendant execution resumes:
        self.send(compas_rrc.PrintText("continue to next location."))


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
