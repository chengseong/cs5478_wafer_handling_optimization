"""
===============================================================================
PHASE 2: Classical Baselines & Analytical Failure Modes (Runs 1 to 4)
===============================================================================
Presentation Slide Purpose:
- Demonstrates why classical robotics approaches fail under non-prehensile friction limits:
  * Run 1 (Decoupled Joint Limits): Kinematic Coupling Fallacy -> 49.35 mm slip
  * Run 2 (Scalar Joint Derating): Scalar Bound Inadequacy -> 19.64 mm slip
  * Run 3 (Cartesian Operational IK): Dynamic Tracking Ringing -> 12.80 mm slip
  * Run 4 (Computed Torque / RNEA): Incomplete Dynamic Decoupling -> 7.22 mm slip
===============================================================================
"""

import time
import argparse
import numpy as np
import pybullet as p
import pybullet_data
import toppra as ta
from toppra import constraint, interpolator
from scara_kinematics import ScaraKinematics


def compute_full_jacobian(kin, q):
    th1, th2, th3 = q
    th12 = th1 + th2
    th123 = th12 + th3

    s1, s12, s123 = np.sin(th1), np.sin(th12), np.sin(th123)
    c1, c12, c123 = np.cos(th1), np.cos(th12), np.cos(th123)

    J = np.array([
        [-kin.l1 * s1 - kin.l2 * s12 - kin.l3 * s123, -kin.l2 * s12 - kin.l3 * s123, -kin.l3 * s123],
        [ kin.l1 * c1 + kin.l2 * c12 + kin.l3 * c123,  kin.l2 * c12 + kin.l3 * c123,  kin.l3 * c123],
        [ 1.0,                                         1.0,                           1.0]
    ])
    return J


def setup_phase2_sim():
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.setPhysicsEngineParameter(numSolverIterations=150, enableConeFriction=1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.1, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.45, -0.05, 0.05]
    )
    return client


def setup_phase2_environment(kin, q_init):
    robot_id = p.loadURDF("scara_wafer.urdf", [0, 0, 0], useFixedBase=True)
    paddle_link_idx = 2
    p.changeDynamics(robot_id, paddle_link_idx, lateralFriction=1.0, restitution=0.0)

    for i in range(3):
        p.resetJointState(robot_id, i, q_init[i])

    w_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.15, height=0.004)
    w_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.15, length=0.002, rgbaColor=[0.2, 0.25, 0.35, 1.0])

    init_fk = kin.forward_kinematics(q_init)
    init_yaw = init_fk[2]
    init_orn = p.getQuaternionFromEuler([0, 0, init_yaw])

    wafer_id = p.createMultiBody(
        baseMass=0.128,
        baseCollisionShapeIndex=w_col,
        baseVisualShapeIndex=w_vis,
        basePosition=[init_fk[0], init_fk[1], 0.092],
        baseOrientation=init_orn
    )
    p.changeDynamics(wafer_id, -1, lateralFriction=0.25, restitution=0.0)

    # Slit Valve aperture enclosure at X = 0.60m
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.25, 0.05])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.25, 0.05], rgbaColor=[0.5, 0.5, 0.6, 0.4])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col,
                      baseVisualShapeIndex=wall_vis, basePosition=[0.60, 0.0, 0.165])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col,
                      baseVisualShapeIndex=wall_vis, basePosition=[0.60, 0.0, 0.015])

    return robot_id, wafer_id, paddle_link_idx


def get_relative_wafer_displacement(robot_id, paddle_link_idx, wafer_id):
    link_state = p.getLinkState(robot_id, paddle_link_idx, computeForwardKinematics=1)
    link_pos = np.array(link_state[0])
    link_orn = link_state[1]

    wafer_pos, _ = p.getBasePositionAndOrientation(wafer_id)
    wafer_pos = np.array(wafer_pos)

    inv_link_pos, inv_link_orn = p.invertTransform(link_pos, link_orn)
    local_wafer_pos, _ = p.multiplyTransforms(inv_link_pos, inv_link_orn, wafer_pos, [0, 0, 0, 1])
    return np.array(local_wafer_pos)


def run_baseline(run_number=4):
    kin = ScaraKinematics(l1=0.40, l2=0.35, l3=0.15)
    setup_phase2_sim()

    cartesian_wps = np.array([
        [0.45, -0.20, 0.0],
        [0.52, -0.10, 0.0],
        [0.60,  0.00, 0.0],
        [0.72,  0.00, 0.0]
    ])

    q_init = kin.inverse_kinematics(cartesian_wps[0])
    robot_id, wafer_id, paddle_idx = setup_phase2_environment(kin, q_init)

    # Plan trajectory based on selected baseline run
    a_crit = 0.25 * 9.81  # 2.4525 m/s^2
    a_max = 0.85 * a_crit # 2.085 m/s^2

    if run_number == 1:
        # Run 1: Decoupled Joint Limits
        jwps = np.array([kin.inverse_kinematics(w) for w in cartesian_wps])
        path = interpolator.SplineInterpolator(np.linspace(0, 1, 4), jwps)
        pc_vel = constraint.JointVelocityConstraint([2.0, 2.0, 4.0])
        pc_acc = constraint.JointAccelerationConstraint([a_max / kin.l1, a_max / kin.l2, a_max / kin.l3])
        inst = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, gridpoints=np.linspace(0, 1, 150), solver_wrapper='ecos')
        traj = inst.compute_trajectory(0.0, 0.0)
        mode = "RUN 1: Decoupled Joint Spline (Kinematic Coupling Fallacy)"
    elif run_number == 2:
        # Run 2: 4x Derated Joint Limits
        jwps = np.array([kin.inverse_kinematics(w) for w in cartesian_wps])
        path = interpolator.SplineInterpolator(np.linspace(0, 1, 4), jwps)
        pc_vel = constraint.JointVelocityConstraint([1.0, 1.0, 2.0])
        pc_acc = constraint.JointAccelerationConstraint([a_max / (4 * kin.l1), a_max / (4 * kin.l2), a_max / (4 * kin.l3)])
        inst = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, gridpoints=np.linspace(0, 1, 150), solver_wrapper='ecos')
        traj = inst.compute_trajectory(0.0, 0.0)
        mode = "RUN 2: 4x Derated Scalar Joint Limits (Scalar Inadequacy)"
    else:
        # Run 3 & 4: Cartesian Spline mapped via IK
        ss_wps = np.linspace(0, 1, len(cartesian_wps))
        path = interpolator.SplineInterpolator(ss_wps, cartesian_wps, bc_type='clamped')
        a_comp = a_max / np.sqrt(2.0)
        pc_vel = constraint.JointVelocityConstraint([1.0, 1.0, 2.0])
        pc_acc = constraint.JointAccelerationConstraint([a_comp, a_comp, 4.0])
        inst = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, gridpoints=np.linspace(0, 1, 150), solver_wrapper='ecos')
        traj = inst.compute_trajectory(0.0, 0.0)
        mode = f"RUN {run_number}: Cartesian Operational IK & Computed Torque"

    print("=" * 65)
    print(f"PHASE 2: {mode}")
    print(f"Trajectory Duration: {traj.duration:.3f} s")
    print("=" * 65)

    # Contact settling phase (120 steps)
    for _ in range(120):
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id, jointIndices=[0, 1, 2], controlMode=p.POSITION_CONTROL,
            targetPositions=q_init, targetVelocities=[0.0, 0.0, 0.0],
            positionGains=[0.8, 0.8, 0.8], velocityGains=[1.0, 1.0, 1.0], forces=[300.0, 200.0, 100.0]
        )
        p.stepSimulation()

    init_local_offset = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
    dt = 1.0 / 240.0
    t = 0.0
    max_slip = 0.0

    while t < traj.duration:
        if run_number in [1, 2]:
            q_des = traj.eval(t)
            qd_des = traj.evald(t)
            p.setJointMotorControlArray(
                bodyUniqueId=robot_id, jointIndices=[0, 1, 2], controlMode=p.POSITION_CONTROL,
                targetPositions=q_des, targetVelocities=qd_des,
                positionGains=[0.8, 0.8, 0.8], velocityGains=[1.0, 1.0, 1.0], forces=[500.0, 400.0, 200.0]
            )
        else:
            x_des = traj.eval(t)
            xd_des = traj.evald(t)
            xdd_des = traj.evaldd(t)
            q_des = kin.inverse_kinematics(x_des)
            J = compute_full_jacobian(kin, q_des)
            qd_des = np.linalg.solve(J, xd_des)

            if run_number == 4:
                J_dot = np.zeros((3, 3))
                J_dot[:2, :] = kin.jacobian_derivative(q_des, qd_des)
                qdd_des = np.linalg.solve(J, xdd_des - J_dot @ qd_des)
                tau_ff = p.calculateInverseDynamics(robot_id, q_des.tolist(), qd_des.tolist(), qdd_des.tolist())
                forces = [abs(tau_ff[i]) + 20.0 for i in range(3)]
            else:
                forces = [300.0, 200.0, 100.0]

            p.setJointMotorControlArray(
                bodyUniqueId=robot_id, jointIndices=[0, 1, 2], controlMode=p.POSITION_CONTROL,
                targetPositions=q_des, targetVelocities=qd_des,
                positionGains=[0.8, 0.8, 0.8], velocityGains=[1.0, 1.0, 1.0], forces=forces
            )

        p.stepSimulation()
        curr_local = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
        max_slip = max(max_slip, np.linalg.norm(curr_local[:2] - init_local_offset[:2]))
        t += dt
        time.sleep(dt * 0.2)

    final_slip_mm = max_slip * 1000.0
    print("\n" + "=" * 65)
    print("PHASE 2 BENCHMARK TELEMETRY")
    print("=" * 65)
    print(f"Mode Tested:               Run {run_number}")
    print(f"Total Trajectory Time:     {traj.duration:.3f} s")
    print(f"Max Wafer Slip Offset:     {final_slip_mm:.4f} mm")
    print(f"Success Threshold:         <= 0.1000 mm")
    print(f"Verification Result:       FAIL (Coulomb Breach Expected & Documented)")
    print("=" * 65 + "\n")

    time.sleep(2.0)
    p.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Phase 2 Classical Baseline Demonstrator")
    parser.add_argument("--run", type=int, default=4, choices=[1, 2, 3, 4], help="Baseline Run (1, 2, 3, or 4)")
    args = parser.parse_args()
    run_baseline(args.run)

