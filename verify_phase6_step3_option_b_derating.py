"""
verify_phase6_step3_option_b_derating.py
=============================================================================
Phase 6 (Step 3 - Option B): Conservative Acceleration Derating & Time Dilation
-----------------------------------------------------------------------------
Investigates the effect of expanding the acceleration margin below the Coulomb
cliff to achieve final slip convergence (<= 0.100 mm / 100 um) with 0 collisions.

Parameters:
- Stage 1 (1D FOUP extraction):    eta_1 = 0.50 (a_max = 1.226 m/s^2)
- Stage 2 (90° aisle rotation):     eta_2 = 0.35 (a_max = 0.858 m/s^2)
- Stage 3 (1D chamber insertion):  eta_3 = 0.50 (a_max = 1.226 m/s^2)
- Control Mode: Closed-loop position control with RNEA feedforward torque bias
  and critically tuned damping (zeta >= 1.0) operating well within the linear regime.
- Stage 3 terminal waypoint: X = 0.705 m (25 mm chamber rear wall clearance).
=============================================================================
"""

import time
import numpy as np
import pybullet as p
import pybullet_data
import toppra as ta

from scara_kinematics import ScaraKinematics
from cartesian_friction_constraint import CartesianFrictionConeConstraint
from verify_phase6_step3_torque_control import (
    build_foup_enclosure,
    build_chamber_enclosure,
    setup_phase6_environment
)

def get_relative_wafer_displacement(robot_id, paddle_link_idx, wafer_id):
    """
    Computes link-relative wafer displacement with computeForwardKinematics=1
    to prevent asynchronous link state phase lag in PyBullet.
    Uses linkWorldPosition (linkState[0]) which is the calibrated center of mass frame.
    """
    link_state = p.getLinkState(robot_id, paddle_link_idx, computeForwardKinematics=1)
    link_pos = np.array(link_state[0])
    link_orn = link_state[1]

    wafer_pos, _ = p.getBasePositionAndOrientation(wafer_id)
    wafer_pos = np.array(wafer_pos)

    inv_link_pos, inv_link_orn = p.invertTransform(link_pos, link_orn)
    local_wafer_pos, _ = p.multiplyTransforms(inv_link_pos, inv_link_orn, wafer_pos, [0, 0, 0, 1])
    return np.array(local_wafer_pos)


def generate_derated_trajectories(kin, mu=0.25, g=9.81, num_facets=16,
                                  eta_linear=0.50, eta_turning=0.35):
    """
    Generates time-optimal trajectories under conservative acceleration derating:
    - Linear segments (Stage 1 & 3): a_max = eta_linear * mu * g ≈ 1.226 m/s^2
    - Turning segment (Stage 2):      a_max = eta_turning * mu * g ≈ 0.858 m/s^2
    Provides a 50% to 65% cushion below the Coulomb cliff (2.4525 m/s^2).
    """
    a_crit = mu * g  # 2.4525 m/s^2
    a_max_linear = eta_linear * a_crit
    a_max_turning = eta_turning * a_crit
    c_vel = ta.constraint.JointVelocityConstraint([2.0, 2.0, 4.0])

    # STAGE 1: 1D FOUP Retraction out of cassette door [0.0, -0.65] -> [0.0, -0.30]
    wps1 = np.array([
        [0.00, -0.65, -np.pi/2],
        [0.00, -0.53, -np.pi/2],
        [0.00, -0.41, -np.pi/2],
        [0.00, -0.30, -np.pi/2]
    ])
    jwps1 = np.array([kin.inverse_kinematics(w) for w in wps1])
    path1 = ta.interpolator.SplineInterpolator(np.linspace(0, 1, len(jwps1)), jwps1)
    c_fric1 = CartesianFrictionConeConstraint(kin, a_max=a_max_linear, num_facets=num_facets, inscribe=True)
    inst1 = ta.algorithm.TOPPRA([c_vel, c_fric1], path1, gridpoints=np.linspace(0, 1, 200), solver_wrapper='ecos')
    traj1 = inst1.compute_trajectory(0.0, 0.0)

    # STAGE 2: 90° Cleanroom Aisle Turn [0.0, -0.30] -> [0.45, 0.0]
    wps2 = np.array([
        [0.00, -0.30, -np.pi/2],
        [0.22, -0.22, -np.pi/3],
        [0.38, -0.10, -np.pi/6],
        [0.45,  0.00,  0.0]
    ])
    jwps2 = np.array([kin.inverse_kinematics(w) for w in wps2])
    path2 = ta.interpolator.SplineInterpolator(np.linspace(0, 1, len(jwps2)), jwps2)
    c_fric2 = CartesianFrictionConeConstraint(kin, a_max=a_max_turning, num_facets=num_facets, inscribe=True)
    inst2 = ta.algorithm.TOPPRA([c_vel, c_fric2], path2, gridpoints=np.linspace(0, 1, 200), solver_wrapper='ecos')
    traj2 = inst2.compute_trajectory(0.0, 0.0)

    # STAGE 3: Pure 1D Slit Valve Insertion [0.45, 0.0] -> [0.705, 0.0] (25mm rear clearance)
    wps3 = np.array([
        [0.45,  0.00, 0.0],
        [0.535, 0.00, 0.0],
        [0.62,  0.00, 0.0],
        [0.705, 0.00, 0.0]
    ])
    jwps3 = np.array([kin.inverse_kinematics(w) for w in wps3])
    path3 = ta.interpolator.SplineInterpolator(np.linspace(0, 1, len(jwps3)), jwps3)
    c_fric3 = CartesianFrictionConeConstraint(kin, a_max=a_max_linear, num_facets=num_facets, inscribe=True)
    inst3 = ta.algorithm.TOPPRA([c_vel, c_fric3], path3, gridpoints=np.linspace(0, 1, 200), solver_wrapper='ecos')
    traj3 = inst3.compute_trajectory(0.0, 0.0)

    return traj1, traj2, traj3, jwps1, jwps2, jwps3


def main():
    kin = ScaraKinematics(l1=0.40, l2=0.35, l3=0.15)

    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.setPhysicsEngineParameter(numSolverIterations=150, enableConeFriction=1)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.55,
        cameraYaw=-40,
        cameraPitch=-30,
        cameraTargetPosition=[0.30, -0.30, 0.08]
    )

    print("\n" + "=" * 75)
    print("PHASE 6 (STEP 3 - OPTION B): CONSERVATIVE DERATING (eta1=0.50, eta2=0.35, eta3=0.50)")
    print("=" * 75)
    traj1, traj2, traj3, jwps1, jwps2, jwps3 = generate_derated_trajectories(kin, num_facets=16)
    dur1, dur2, dur3 = traj1.duration, traj2.duration, traj3.duration
    total_duration = dur1 + dur2 + dur3
    print(f"Stage 1 Duration: {dur1:.3f} s (a_max = {0.50 * 0.25 * 9.81:.3f} m/s^2)")
    print(f"Stage 2 Duration: {dur2:.3f} s (a_max = {0.35 * 0.25 * 9.81:.3f} m/s^2)")
    print(f"Stage 3 Duration: {dur3:.3f} s (a_max = {0.50 * 0.25 * 9.81:.3f} m/s^2)")
    print(f"Total Trajectory Duration: {total_duration:.3f} s")

    q_init = jwps1[0]
    robot_id, wafer_id, paddle_link_idx, obstacle_ids = setup_phase6_environment(kin, q_init)

    # 120-step contact settling phase in position control
    for _ in range(120):
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=[0, 1, 2],
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_init,
            forces=[500, 400, 200]
        )
        p.stepSimulation()

    init_local_offset = get_relative_wafer_displacement(robot_id, paddle_link_idx, wafer_id)
    print(f"Initial Settling Wafer Offset: {np.linalg.norm(init_local_offset[:2]) * 1e6:.3f} um")

    dt = 1.0 / 240.0
    max_slip = 0.0
    collision_events = []

    def check_collisions(current_t):
        cps = p.getContactPoints()
        for c in cps:
            if c[1] in obstacle_ids or c[2] in obstacle_ids:
                if not any(e["time"] == round(current_t, 3) for e in collision_events):
                    collision_events.append({
                        "time": round(current_t, 3), "force": c[9],
                        "body_a": c[1], "body_b": c[2], "link_a": c[3], "link_b": c[4]
                    })
                    print(f"  [COLLISION at t={current_t:.3f}s] BodyA={c[1]} (link {c[3]}) vs BodyB={c[2]} (link {c[4]}), Force: {c[9]:.2f} N")

    def execute_stage(stage_num, traj, stage_duration, t_offset):
        nonlocal max_slip
        t = 0.0
        while t < stage_duration:
            q_des = traj.eval(t)
            qd_des = traj.evald(t)
            qdd_des = traj.evaldd(t)

            # RNEA feedforward torque bias
            tau_ff = p.calculateInverseDynamics(robot_id, q_des.tolist(), qd_des.tolist(), qdd_des.tolist())
            max_torque_clamp = [abs(tau_ff[i]) + 25.0 for i in range(3)]

            # Position control with RNEA feedforward bias
            p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[0, 1, 2],
                controlMode=p.POSITION_CONTROL,
                targetPositions=q_des,
                targetVelocities=qd_des,
                positionGains=[0.75, 0.75, 0.75],
                velocityGains=[1.3, 1.3, 1.3],
                forces=max_torque_clamp
            )

            p.stepSimulation()
            check_collisions(t_offset + t)
            curr_local = get_relative_wafer_displacement(robot_id, paddle_link_idx, wafer_id)
            max_slip = max(max_slip, np.linalg.norm(curr_local[:2] - init_local_offset[:2]))
            t += dt
            time.sleep(dt * 0.1)

    print("\nExecuting Stage 1: 1D FOUP Retraction (Derated)...")
    execute_stage(1, traj1, dur1, 0.0)
    slip_stage1 = max_slip * 1000.0
    print(f"Stage 1 Complete. Wafer Slip at Aisle Door: {slip_stage1:.4f} mm")

    print("\nExecuting Stage 2: 90° Cleanroom Aisle Transfer (Derated)...")
    execute_stage(2, traj2, dur2, dur1)
    slip_stage2 = max_slip * 1000.0
    print(f"Stage 2 Complete. Wafer Slip at Pre-Alignment Gate: {slip_stage2:.4f} mm")

    print("\nExecuting Stage 3: Pure 1D Slit Valve Insertion (Derated)...")
    execute_stage(3, traj3, dur3, dur1 + dur2)

    # Post-arrival settling (60 steps)
    for _ in range(60):
        p.stepSimulation()

    curr_local = get_relative_wafer_displacement(robot_id, paddle_link_idx, wafer_id)
    max_slip = max(max_slip, np.linalg.norm(curr_local[:2] - init_local_offset[:2]))

    final_slip_mm = max_slip * 1000.0
    stage3_added_slip = final_slip_mm - slip_stage2

    print("\n" + "=" * 75)
    print("PHASE 6 (STEP 3 - OPTION B): CONSERVATIVE DERATING RESULTS")
    print("=" * 75)
    print(f"Stage 1 (FOUP Retraction along +Y):   {dur1:.3f} s  | Slip: {slip_stage1:.4f} mm")
    print(f"Stage 2 (90° Aisle Transfer):        {dur2:.3f} s  | Slip: {slip_stage2:.4f} mm")
    print(f"Stage 3 (Slit Valve & Chamber In.):   {dur3:.3f} s  | Stage Slip Added: {stage3_added_slip:.4f} mm")
    print(f"TOTAL EFEM CYCLE DURATION:           {total_duration:.3f} s")
    print(f"CUMULATIVE PEAK WAFER SLIP:          {final_slip_mm:.4f} mm ({final_slip_mm * 1000:.1f} um)")
    print(f"TOTAL OBSTACLE COLLISIONS DETECTED:  {len(collision_events)}")
    print(f"Comparison vs Run 7 (Baseline RNEA): {final_slip_mm:.4f} mm vs 2.4412 mm")
    print(f"Comparison vs Run 9 (Torque Mode):   {final_slip_mm:.4f} mm vs 4.7947 mm")
    if final_slip_mm <= 0.100:
        print("VERDICT: PASS (Meets semiconductor spec <= 0.100 mm / 100 um)")
    else:
        print(f"VERDICT: Slip = {final_slip_mm:.4f} mm (Spec: <= 0.100 mm)")
    print("=" * 75 + "\n")

    time.sleep(2.0)
    p.disconnect()


if __name__ == "__main__":
    main()

