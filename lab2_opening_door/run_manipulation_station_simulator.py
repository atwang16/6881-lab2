import argparse
from manipulation_station_simulator import ManipulationStationSimulator

import numpy as np
from pydrake.multibody import inverse_kinematics
from pydrake.trajectories import (
    PiecewisePolynomial
)
from pydrake.util.eigen_geometry import Isometry3
from pydrake.math import RollPitchYaw, RotationMatrix
from robot_plans import *

from pydrake.common import FindResourceOrThrow
import matplotlib.pyplot as plt
from manipulation_station_plan_runner import *


'''
Plots iiwa_external_torque from its signal logger system. 
'''
def PlotExternalTorqueLog(iiwa_external_torque_log):
    fig_external_torque = plt.figure(figsize=(8, 18))
    t = iiwa_external_torque_log.sample_times()
    for i, torque in enumerate(iiwa_external_torque_log.data()):
        ax = fig_external_torque.add_subplot(711 + i)
        ax.plot(t, torque, label='torque_external@joint_%d' % (i + 1))
        ax.set_xlabel("t(s)")
        ax.set_ylabel("N/m")
        ax.legend()
        ax.grid(True)

    plt.tight_layout()
    plt.show()

def PlotIiwaPositionLog(iiwa_position_command_log, iiwa_position_measured_log):
    fig = plt.figure(figsize=(8, 18))
    t = iiwa_position_command_log.sample_times()
    for i in range(len(iiwa_position_command_log.data())):
        ax = fig.add_subplot(711 + i)
        q_commanded = iiwa_position_command_log.data()[i]
        q_measured = iiwa_position_measured_log.data()[i]
        ax.plot(t, q_commanded/np.pi*180, label='q_commanded@joint_%d' % (i + 1))
        ax.plot(t, q_measured/np.pi*180, label='q_measrued@joint_%d' % (i + 1))
        ax.set_xlabel("t(s)")
        ax.set_ylabel("degrees")
        ax.legend()
        ax.grid(True)

    plt.tight_layout()
    plt.show()

def GenerateIiwaPlansAndGripperSetPoints(manip_station_sim, is_printing=True):
    plant = manip_station_sim.plant
    tree = plant.tree()
    iiwa_model = plant.GetModelInstanceByName("iiwa")
    gripper_model = plant.GetModelInstanceByName("gripper")

    # get "home" pose
    ik_scene = inverse_kinematics.InverseKinematics(plant)
    world_frame = plant.world_frame()
    gripper_frame = plant.GetFrameByName("body", gripper_model)

    theta_bound = 0.005 * np.pi # 0.9 degrees
    X_EEa = GetEndEffectorWorldAlignedFrame()
    R_EEa = RotationMatrix(X_EEa.rotation())

    ik_scene.AddOrientationConstraint(
        frameAbar=world_frame, R_AbarA=R_WEa_ref,
        frameBbar=gripper_frame, R_BbarB=R_EEa,
        theta_bound=theta_bound)

    p_WQ0 = p_WQ_home
    p_WQ_lower = p_WQ0 - 0.005
    p_WQ_upper = p_WQ0 + 0.005
    ik_scene.AddPositionConstraint(
        frameB=gripper_frame, p_BQ=p_EQ,
        frameA=world_frame,
        p_AQ_lower=p_WQ_lower, p_AQ_upper=p_WQ_upper)

    prog = ik_scene.prog()
    prog.SetInitialGuess(ik_scene.q(), np.zeros(plant.num_positions()))
    result = prog.Solve()
    if is_printing:
        print result
    q_val_0 = prog.GetSolution(ik_scene.q())

    # q returned by IK consists of the configuration of all bodies, including
    # the iiwa arm, the box, the gripper and the bottle.
    # But the trajectory sent to iiwa only needs the configuration of iiwa.
    # This function takes in an array of shape (n, plant.num_positions()),
    # and returns an array of shape (n, 7), which only has the configuration of the iiwa arm.
    def GetKukaQKnots(q_knots):
        if len(q_knots.shape) == 1:
            q_knots.resize(1, q_knots.size)
        n = q_knots.shape[0]
        q_knots_kuka = np.zeros((n, 7))
        for i, q_knot in enumerate(q_knots):
            q_knots_kuka[i] = tree.get_positions_from_array(iiwa_model, q_knot)

        return q_knots_kuka


    def InterpolateStraightLine(p_WQ_start, p_WQ_end, num_knot_points, i):
        return (p_WQ_end - p_WQ_start)/num_knot_points*(i+1) + p_WQ_start

    # inverse_kin_ponitwise
    def GoFromPointToPoint(p_WQ_start, p_WQ_end, duration,
                           num_knot_points,
                           q_initial_guess,
                           InterpolatePosition=InterpolateStraightLine,
                           position_tolerance=0.005):
        # The first knot point is the zero posture.
        # The second knot is the pre-pre-grasp posture q_val_0
        # The rest are solved for in the for loop below.
        # The hope is that using more knot points makes the trajectory
        # smoother.
        q_knots = np.zeros((num_knot_points+1, plant.num_positions()))
        q_knots[0] = q_initial_guess

        for i in range(num_knot_points):
            ik = inverse_kinematics.InverseKinematics(plant)
            q_variables = ik.q()

            # ik.AddOrientationConstraint(
            #     frameAbar=world_frame, R_AbarA=R_WEa_ref,
            #     frameBbar=gripper_frame, R_BbarB=R_EEa,
            #     theta_bound=theta_bound)

            p_WQ = InterpolatePosition(p_WQ_start, p_WQ_end, num_knot_points, i)

            ik.AddPositionConstraint(
                frameB=gripper_frame, p_BQ=p_EQ,
                frameA=world_frame,
                p_AQ_lower=p_WQ - position_tolerance,
                p_AQ_upper=p_WQ + position_tolerance)

            prog = ik.prog()
            # use the robot posture at the previous knot point as
            # an initial guess.
            prog.SetInitialGuess(q_variables, q_knots[i])
            result = prog.Solve()
            if is_printing:
                print i, ": ", result
            q_knots[i+1] = prog.GetSolution(q_variables)

        t_knots = np.linspace(0, duration, num_knot_points + 1)

        q_knots_kuka = GetKukaQKnots(q_knots)
        qtraj = PiecewisePolynomial.Cubic(
            t_knots, q_knots_kuka.T,
            np.zeros(7), np.zeros(7))

        return qtraj, q_knots

    # Generating trajectories
    num_knot_points = 10

    # move to grasp left door handle
    p_WQ_start = p_WQ0
    p_WQ_end = p_WC_handle
    qtraj_move_to_handle, q_knots_full = GoFromPointToPoint(
        p_WQ_start, p_WQ_end, duration=5.0,
        num_knot_points=num_knot_points, q_initial_guess=q_val_0,
        position_tolerance=0.001)

    # close gripper
    q_knots = np.zeros((2, 7))
    q_knots[0] = qtraj_move_to_handle.value(qtraj_move_to_handle.end_time()).squeeze()
    qtraj_close_gripper = PiecewisePolynomial.ZeroOrderHold([0, 1], q_knots.T)

    # pull handle along an arc
    def InterpolateArc(angle_start, angle_end, num_knot_points, i):
        radius = r_handle
        theta = angle_start + (angle_end - angle_start)*(i+1)/num_knot_points
        return p_WC_left_hinge + [-radius * np.sin(theta), -radius * np.cos(theta), 0]

    angle_start = theta0_hinge
    angle_end = np.pi/180*60
    qtraj_pull_handle, q_knots_full = GoFromPointToPoint(
        angle_start, angle_end, duration=5.0, num_knot_points=20,
        q_initial_guess=q_knots_full[-1], InterpolatePosition=InterpolateArc,
        position_tolerance=0.002)

    q_traj_list = [qtraj_move_to_handle,
                   qtraj_close_gripper,
                   qtraj_pull_handle]

    plan_list = []
    for q_traj in q_traj_list:
        plan_list.append(JointSpacePlan(q_traj))

    gripper_setpoint_list = [0.03, 0.0, 0.0]
    return plan_list, gripper_setpoint_list


if __name__ == '__main__':
    # define command line arguments
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--hardware", action='store_true',
        help="Use the ManipulationStationHardwareInterface instead of an "
             "in-process simulation.")
    args = parser.parse_args()
    is_hardware = args.hardware

    object_file_path = FindResourceOrThrow(
            "drake/examples/manipulation_station/models/061_foam_brick.sdf")

    manip_station_sim = ManipulationStationSimulator(
        time_step=2e-3,
        object_file_path=object_file_path,
        object_base_link_name="base_link",
        is_hardware=is_hardware)

    # Generate plans.
    q0 = [0, 0, 0, -1.75, 0, 1.0, 0]
    plan_list, gripper_setpoint_list = \
        GenerateIiwaPlansAndGripperSetPoints(manip_station_sim, q0)

    # Add the position/impedance plan that opens the left door.
    plan_list.append(OpenLeftDoorPositionPlan(
        angle_start=theta0_hinge, angle_end=np.pi/4, duration=6.0))
    gripper_setpoint_list.append(0.)

    if is_hardware:
        iiwa_position_measured_log, iiwa_external_torque_log, wsg_state_log, wsg_command_log = \
            manip_station_sim.RunRealRobot(plan_list, gripper_setpoint_list)
        PlotExternalTorqueLog(iiwa_external_torque_log)
    else:
        iiwa_position_command_log, iiwa_position_measured_log, iiwa_external_torque_log, \
                wsg_state_log, wsg_command_log = \
            manip_station_sim.RunSimulation(plan_list, gripper_setpoint_list,
                                        extra_time=2.0, real_time_rate=1.0, q0_kuka=q0)
        PlotExternalTorqueLog(iiwa_external_torque_log)
        PlotIiwaPositionLog(iiwa_position_command_log, iiwa_position_measured_log)

