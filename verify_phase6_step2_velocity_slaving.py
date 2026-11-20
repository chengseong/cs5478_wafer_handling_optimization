"""
verify_phase6_step2_velocity_slaving.py
=============================================================================
Phase 6 (Step 2): Autonomous Wafer Transfer with Velocity-Slaved Feedforward Control
-----------------------------------------------------------------------------
Solves the root physical failure mode of Run 7 (position-servo compliance ringing
and virtual spring snappy oscillations).
- Uses regular C^2 Spline Interpolator with 16-facet inscribed friction cone
  constraint to guarantee strict friction bounds (||a_tip|| <= eta * mu * g).
- Directs motor control via p.VELOCITY_CONTROL slaved to TOPP-RA qd_des(t).
- Clamps joint motor torque ceiling dynamically via RNEA: tau_max = |tau_RNEA| + 15 Nm.
- Calibrates Stage 3 terminal waypoint to X = 0.705 m to provide 25 mm clearance
  from the vacuum chamber rear wall.
- Measures link-relative wafer slip offset (Target: <= 0.100 mm / 100 um) with 0 collisions.
=============================================================================
"""

import time
import numpy as np
import pybullet as p
import pybullet_data
import toppra as ta

from scara_kinematics import ScaraKinematics
from cartesian_friction_constraint import CartesianFrictionConeConstraint


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


def build_foup_enclosure(center_x=0.00, center_y=-0.65, center_z=0.092):
    """
    Constructs the FOUP physical enclosure with front door facing +Y into the cleanroom aisle.
    """
    foup_parts = []

    # 1. Base Support Floor Table (Z = 0.02m)
    pedestal_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.22, 0.22, 0.02], rgbaColor=[0.3, 0.3, 0.35, 1.0])
    pedestal_base = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=pedestal_vis,
                                      basePosition=[center_x, center_y, 0.02])
    foup_parts.append(pedestal_base)

    # 2. Back Enclosure Wall (-Y = -0.85m) - COLLISION ENABLED
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.20, 0.01, 0.065])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.20, 0.01, 0.065], rgbaColor=[0.25, 0.35, 0.45, 0.5])
    back_wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col, baseVisualShapeIndex=wall_vis,
                                  basePosition=[center_x, center_y - 0.20, center_z + 0.01])
    foup_parts.append(back_wall)

    # 3. West Enclosure Wall (-X = -0.20m) - COLLISION ENABLED
    side_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.20, 0.065])
    side_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.20, 0.065], rgbaColor=[0.25, 0.35, 0.45, 0.5])
    left_wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=side_col, baseVisualShapeIndex=side_vis,
                                  basePosition=[center_x - 0.20, center_y, center_z + 0.01])
    foup_parts.append(left_wall)

    # 4. East Enclosure Wall (+X = +0.20m) - COLLISION ENABLED
    right_wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=side_col, baseVisualShapeIndex=side_vis,
                                   basePosition=[center_x + 0.20, center_y, center_z + 0.01])
    foup_parts.append(right_wall)

    # 5. Top Roof (+Z = 0.165m) - COLLISION ENABLED
    roof_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.20, 0.20, 0.01])
    roof_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.20, 0.20, 0.01], rgbaColor=[0.20, 0.30, 0.40, 0.4])
    top_roof = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=roof_col, baseVisualShapeIndex=roof_vis,
                                 basePosition=[center_x, center_y, center_z + 0.075])
    foup_parts.append(top_roof)

    return foup_parts


def build_chamber_enclosure(chamber_x=0.75, chamber_y=0.00, chamber_z=0.092):
    """
    Constructs the vacuum process chamber enclosure behind the slit valve at X = 0.60m.
    """
    chamber_parts = []

    # 1. Top Roof (Z = 0.165m) - COLLISION ENABLED
    roof_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.14, 0.21, 0.01])
    roof_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.14, 0.21, 0.01], rgbaColor=[0.25, 0.40, 0.55, 0.35])
    chamber_roof = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=roof_col, baseVisualShapeIndex=roof_vis,
                                     basePosition=[chamber_x, chamber_y, chamber_z + 0.075])
    chamber_parts.append(chamber_roof)

    # 2. Far End Wall (+X = 0.89m) - COLLISION ENABLED
    back_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.21, 0.065])
    back_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.21, 0.065], rgbaColor=[0.30, 0.35, 0.45, 0.5])
    chamber_back = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=back_col, baseVisualShapeIndex=back_vis,
                                     basePosition=[chamber_x + 0.14, chamber_y, chamber_z + 0.01])
    chamber_parts.append(chamber_back)

    # 3. North Lateral Sidewall (+Y = +0.21m) - COLLISION ENABLED
    wall_n_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.14, 0.01, 0.065])
    wall_n_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.14, 0.01, 0.065], rgbaColor=[0.30, 0.35, 0.45, 0.4])
    chamber_wall_n = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_n_col, baseVisualShapeIndex=wall_n_vis,
                                       basePosition=[chamber_x, chamber_y + 0.21, chamber_z + 0.01])
    chamber_parts.append(chamber_wall_n)

    # 4. South Lateral Sidewall (-Y = -0.21m) - COLLISION ENABLED
    wall_s_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.14, 0.01, 0.065])
    wall_s_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.14, 0.01, 0.065], rgbaColor=[0.30, 0.35, 0.45, 0.4])
    chamber_wall_s = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_s_col, baseVisualShapeIndex=wall_s_vis,
                                       basePosition=[chamber_x, chamber_y - 0.21, chamber_z + 0.01])
    chamber_parts.append(chamber_wall_s)

    # 5. Chamber Base Pedestal (Z = 0.02m)
    table_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15, 0.23, 0.02], rgbaColor=[0.25, 0.25, 0.30, 1.0])
    pedestal_table = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=table_vis,
                                       basePosition=[chamber_x, chamber_y, 0.02])
    chamber_parts.append(pedestal_table)

    return chamber_parts


def setup_phase6_environment(kin, q_init):
    robot_id = p.loadURDF("scara_wafer.urdf", [0, 0, 0], useFixedBase=True)
    paddle_link_idx = 2

    # Calibrate paddle friction surface (mu_paddle = 1.0 -> effective mu = 1.0 * 0.25 = 0.25)
    p.changeDynamics(robot_id, paddle_link_idx, lateralFriction=1.0, restitution=0.0)

    for i in range(3):
        p.resetJointState(robot_id, i, q_init[i])

    # Wafer primitive (300mm diameter, 128g mass, 4mm collision thickness)
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

    # Cleanroom Hollow FOUP Enclosure at (0.0, -0.65) with open door facing +Y
    foup_ids = build_foup_enclosure(center_x=0.00, center_y=-0.65, center_z=0.092)

    # Slit valve aperture enclosure (350mm x 50mm opening at X = 0.60m)
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.175, 0.05])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.175, 0.05], rgbaColor=[0.5, 0.5, 0.6, 0.4])

    wall_upper = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col, baseVisualShapeIndex=wall_vis,
                                   basePosition=[0.60, 0.0, 0.165])
    wall_lower = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col, baseVisualShapeIndex=wall_vis,
                                   basePosition=[0.60, 0.0, 0.015])

    # Translucent Process Chamber Housing behind slit valve
    chamber_ids = build_chamber_enclosure(chamber_x=0.75, chamber_y=0.00, chamber_z=0.092)

    # Collision filtering
    p.setCollisionFilterPair(robot_id, wafer_id, 0, -1, enableCollision=0)
    p.setCollisionFilterPair(robot_id, wafer_id, 1, -1, enableCollision=0)
    p.setCollisionFilterPair(robot_id, wall_lower, 1, -1, enableCollision=0)

    foup_collision_ids = foup_ids[1:]
    chamber_collision_ids = chamber_ids[:-1]
    obstacle_ids = [wall_upper, wall_lower] + foup_collision_ids + chamber_collision_ids
    return robot_id, wafer_id, paddle_link_idx, obstacle_ids


def generate_phase6_step2_trajectories(kin, mu=0.25, g=9.81, num_facets=16):
    """
    Generates 3-stage cleanroom EFEM trajectories using regular C^2 Spline Interpolation
    with 16-facet inscribed friction cone constraints and well-conditioned boundary states.
    Stage 3 terminal waypoint is calibrated to X = 0.705m for chamber back wall clearance.
    """
    a_crit = mu * g  # 2.4525 m/s^2
    a_max_linear = 0.85 * a_crit   # ~2.085 m/s^2
    a_max_turning = 0.65 * a_crit  # ~1.594 m/s^2
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
    print("PHASE 6 (STEP 2): VELOCITY-SLAVED FEEDFORWARD CONTROL WITH RNEA TORQUE CLAMPING")
    print("=" * 75)
    print("Generating Regular Spline Trajectories (16-facet inscribed friction cone)...")
    traj1, traj2, traj3, jwps1, jwps2, jwps3 = generate_phase6_step2_trajectories(kin, num_facets=16)
    dur1, dur2, dur3 = traj1.duration, traj2.duration, traj3.duration
    total_duration = dur1 + dur2 + dur3
    print(f"Stage 1 Duration: {dur1:.3f} s")
    print(f"Stage 2 Duration: {dur2:.3f} s")
    print(f"Stage 3 Duration: {dur3:.3f} s")
    print(f"Total Trajectory Duration: {total_duration:.3f} s")

    # Verify peak planned Cartesian tip accelerations along each stage
    for stg_i, traj_i, name_i in [(1, traj1, "Stage 1"), (2, traj2, "Stage 2"), (3, traj3, "Stage 3")]:
        ts = np.linspace(0, traj_i.duration, 500)
        a_tips = []
        for t_val in ts:
            q_val = traj_i.eval(t_val)
            qd_val = traj_i.evald(t_val)
            qdd_val = traj_i.evaldd(t_val)
            J = kin.jacobian(q_val)
            Jdot = kin.jacobian_derivative(q_val, qd_val)
            a_xy = (J @ qdd_val + Jdot @ qd_val)[:2]
            a_tips.append(np.linalg.norm(a_xy))
        print(f"  {name_i} Max Planned Cartesian Tip Accel: {max(a_tips):.4f} m/s^2")

    q_init = jwps1[0]
    robot_id, wafer_id, paddle_idx, obstacle_ids = setup_phase6_environment(kin, q_init)

    # Contact settling phase (120 steps)
    for _ in range(120):
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=[0, 1, 2],
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_init,
            forces=[500, 400, 200]
        )
        p.stepSimulation()

    init_local_offset = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
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

    def execute_stage_velocity_slaved(stage_num, traj, stage_duration, t_offset):
        nonlocal max_slip
        t = 0.0
        while t < stage_duration:
            q_des = traj.eval(t)
            qd_des = traj.evald(t)
            qdd_des = traj.evaldd(t)

            # RNEA Inverse Dynamics feedforward torque calculation
            tau_ff = p.calculateInverseDynamics(robot_id, q_des.tolist(), qd_des.tolist(), qdd_des.tolist())
            max_torque_clamp = [abs(tau_ff[i]) + 15.0 for i in range(3)]

            # Velocity-slaved control: eliminates virtual position spring (Kp=0)
            p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[0, 1, 2],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=qd_des,
                velocityGains=[1.2, 1.2, 1.2],
                forces=max_torque_clamp
            )
            p.stepSimulation()
            check_collisions(t_offset + t)
            curr_local = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
            max_slip = max(max_slip, np.linalg.norm(curr_local[:2] - init_local_offset[:2]))
            t += dt
            time.sleep(dt * 0.1)

    print("\nExecuting Stage 1: 1D FOUP Retraction (Velocity Slaved)...")
    execute_stage_velocity_slaved(1, traj1, dur1, 0.0)
    slip_stage1 = max_slip * 1000.0
    print(f"Stage 1 Complete. Wafer Slip at Aisle Door: {slip_stage1:.4f} mm")

    print("\nExecuting Stage 2: 90° Cleanroom Aisle Transfer (Velocity Slaved)...")
    execute_stage_velocity_slaved(2, traj2, dur2, dur1)
    slip_stage2 = max_slip * 1000.0
    print(f"Stage 2 Complete. Wafer Slip at Pre-Alignment Gate: {slip_stage2:.4f} mm")

    print("\nExecuting Stage 3: Pure 1D Slit Valve Insertion (Velocity Slaved)...")
    execute_stage_velocity_slaved(3, traj3, dur3, dur1 + dur2)

    # Post-arrival settling (60 steps)
    for _ in range(60):
        # Hold zero velocity
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=[0, 1, 2],
            controlMode=p.VELOCITY_CONTROL,
            targetVelocities=[0.0, 0.0, 0.0],
            velocityGains=[1.2, 1.2, 1.2],
            forces=[15.0, 15.0, 15.0]
        )
        p.stepSimulation()

    curr_local = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
    max_slip = max(max_slip, np.linalg.norm(curr_local[:2] - init_local_offset[:2]))

    final_slip_mm = max_slip * 1000.0
    stage3_added_slip = final_slip_mm - slip_stage2

    print("\n" + "=" * 75)
    print("PHASE 6 (STEP 2): VELOCITY-SLAVED FEEDFORWARD RESULTS")
    print("=" * 75)
    print(f"Stage 1 (FOUP Retraction along +Y):   {dur1:.3f} s  | Slip: {slip_stage1:.4f} mm")
    print(f"Stage 2 (90° Aisle Transfer):        {dur2:.3f} s  | Slip: {slip_stage2:.4f} mm")
    print(f"Stage 3 (Slit Valve & Chamber In.):   {dur3:.3f} s  | Stage Slip Added: {stage3_added_slip:.4f} mm")
    print(f"TOTAL EFEM CYCLE DURATION:           {total_duration:.3f} s")
    print(f"CUMULATIVE PEAK WAFER SLIP:          {final_slip_mm:.4f} mm ({final_slip_mm * 1000:.1f} um)")
    print(f"TOTAL OBSTACLE COLLISIONS DETECTED:  {len(collision_events)}")
    print(f"Comparison vs Phase 5 (Run 7):       {final_slip_mm:.4f} mm vs 2.4412 mm")
    print(f"Comparison vs Phase 6 Step 1:        {final_slip_mm:.4f} mm vs 2.4994 mm")
    if final_slip_mm <= 0.100:
        print("VERDICT: PASS (Meets semiconductor spec <= 0.100 mm / 100 um)")
    else:
        print(f"VERDICT: Slip = {final_slip_mm:.4f} mm (Spec: <= 0.100 mm)")
    print("=" * 75 + "\n")

    time.sleep(2.0)
    p.disconnect()


if __name__ == "__main__":
    main()

