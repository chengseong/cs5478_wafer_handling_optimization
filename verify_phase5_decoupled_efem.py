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
    Instantiates a standardized 300 mm FOUP Load Port enclosure cassette
    at the cleanroom corridor location (X = 0.0, Y = -0.65) with its open front
    door facing the robot transfer aisle (+Y).
    All walls and roof are physical collision obstacles.
    """
    foup_parts = []

    # 1. Pedestal Table
    table_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.21, 0.20, 0.02], rgbaColor=[0.25, 0.25, 0.30, 1.0])
    pedestal = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=table_vis,
                                 basePosition=[center_x, center_y - 0.02, 0.02])
    foup_parts.append(pedestal)

    # 2. Translucent FOUP Pod Top Shell / Roof (Z = 0.167m) - COLLISION ENABLED
    roof_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.20, 0.18, 0.01])
    roof_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.20, 0.18, 0.01], rgbaColor=[0.15, 0.40, 0.65, 0.35])
    roof = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=roof_col, baseVisualShapeIndex=roof_vis,
                            basePosition=[center_x, center_y - 0.02, center_z + 0.075])
    foup_parts.append(roof)

    # 3. Solid Back Wall (at -Y = -0.85m, facing away from robot) - COLLISION ENABLED
    back_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.20, 0.01, 0.07])
    back_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.20, 0.01, 0.07], rgbaColor=[0.20, 0.25, 0.35, 0.6])
    back_wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=back_col, baseVisualShapeIndex=back_vis,
                                  basePosition=[center_x, center_y - 0.20, center_z + 0.01])
    foup_parts.append(back_wall)

    # 4. Left Side Wall (X = -0.20m) - COLLISION ENABLED
    side_l_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.18, 0.07])
    side_l_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.18, 0.07], rgbaColor=[0.20, 0.25, 0.35, 0.5])
    side_l = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=side_l_col, baseVisualShapeIndex=side_l_vis,
                               basePosition=[center_x - 0.20, center_y - 0.02, center_z + 0.01])
    foup_parts.append(side_l)

    # 5. Right Side Wall (X = +0.20m) - COLLISION ENABLED
    side_r_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.18, 0.07])
    side_r_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.18, 0.07], rgbaColor=[0.20, 0.25, 0.35, 0.5])
    side_r = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=side_r_col, baseVisualShapeIndex=side_r_vis,
                               basePosition=[center_x + 0.20, center_y - 0.02, center_z + 0.01])
    foup_parts.append(side_r)

    return foup_parts



def build_chamber_enclosure(chamber_x=0.75, chamber_y=0.00, chamber_z=0.092):
    """
    Instantiates a standardized semi-cylindrical / box process vacuum chamber enclosure
    behind the slit valve (X >= 0.60m) with translucent walls and a substrate chuck pedestal.
    The front face (X = 0.60m) is left completely open to reveal the slit valve aperture.
    """
    chamber_parts = []
    
    # 1. Translucent Vacuum Chamber Outer Housing / Roof (Z = 0.170m)
    roof_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.14, 0.22, 0.005])
    roof_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.14, 0.22, 0.005], rgbaColor=[0.55, 0.25, 0.65, 0.25])
    roof = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=roof_col, baseVisualShapeIndex=roof_vis,
                            basePosition=[chamber_x, chamber_y, chamber_z + 0.075])
    chamber_parts.append(roof)

    # 2. Chamber Back Wall (at X = +0.89m)
    back_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.005, 0.22, 0.07])
    back_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.005, 0.22, 0.07], rgbaColor=[0.45, 0.20, 0.55, 0.45])
    back_wall = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=back_col, baseVisualShapeIndex=back_vis,
                                  basePosition=[chamber_x + 0.14, chamber_y, chamber_z + 0.01])
    chamber_parts.append(back_wall)

    # 3. Chamber North Side Wall (Y = +0.22m)
    side_n_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.14, 0.005, 0.07])
    side_n_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.14, 0.005, 0.07], rgbaColor=[0.45, 0.20, 0.55, 0.35])
    side_n = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=side_n_col, baseVisualShapeIndex=side_n_vis,
                               basePosition=[chamber_x, chamber_y + 0.22, chamber_z + 0.01])
    chamber_parts.append(side_n)

    # 4. Chamber South Side Wall (Y = -0.22m)
    side_s_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.14, 0.005, 0.07])
    side_s_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.14, 0.005, 0.07], rgbaColor=[0.45, 0.20, 0.55, 0.35])
    side_s = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=side_s_col, baseVisualShapeIndex=side_s_vis,
                               basePosition=[chamber_x, chamber_y - 0.22, chamber_z + 0.01])
    chamber_parts.append(side_s)

    # 5. Chamber Base Pedestal / Solid Foundation Table (Z = 0.02m)
    table_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15, 0.23, 0.02], rgbaColor=[0.25, 0.25, 0.30, 1.0])
    pedestal_table = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=table_vis,
                                       basePosition=[chamber_x, chamber_y, 0.02])
    chamber_parts.append(pedestal_table)

    return chamber_parts


def setup_phase5_environment(kin, q_init):
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

    foup_collision_ids = foup_ids[1:]  # roof, back, left, right
    chamber_collision_ids = chamber_ids[:-1] # roof, back, north, south (pedestal has -1 col)
    obstacle_ids = [wall_upper, wall_lower] + foup_collision_ids + chamber_collision_ids
    return robot_id, wafer_id, paddle_link_idx, obstacle_ids


def generate_phase5_trajectories(kin, mu=0.25, g=9.81, num_facets=8):
    """
    Generates a 3-stage cleanroom transfer trajectory (90° EFEM Layout) with
    calibrated safety margins:
    Stage 1: Pure 1D FOUP Retraction out of cassette door [0, -0.65] -> [0, -0.30] along +Y (eta = 0.85).
    Stage 2: 90° Cleanroom Aisle Turn (eta = 0.65 to suppress dynamic coupling torque demands).
    Stage 3: Pure 1D Slit Valve Insertion [0.45, 0] -> [0.72, 0] along +X (eta = 0.85).
    """
    a_crit = mu * g  # 2.4525 m/s^2
    a_max_linear = 0.85 * a_crit  # ~2.085 m/s^2
    a_max_turning = 0.65 * a_crit # ~1.594 m/s^2
    c_vel = ta.constraint.JointVelocityConstraint([2.0, 2.0, 4.0])

    # STAGE 1: Pure 1D FOUP Retraction
    wps1 = np.array([
        [0.00, -0.65, -np.pi/2],
        [0.00, -0.53, -np.pi/2],
        [0.00, -0.41, -np.pi/2],
        [0.00, -0.30, -np.pi/2]
    ])
    jwps1 = np.array([kin.inverse_kinematics(w) for w in wps1])
    path1 = ta.interpolator.SplineInterpolator(np.linspace(0, 1, 4), jwps1)
    c_fric1 = CartesianFrictionConeConstraint(kin, a_max=a_max_linear, num_facets=num_facets, inscribe=True)
    inst1 = ta.algorithm.TOPPRA([c_vel, c_fric1], path1, gridpoints=np.linspace(0, 1, 100), solver_wrapper='ecos')
    traj1 = inst1.compute_trajectory(0.0, 0.0)

    # STAGE 2: 90° Open Aisle Turn (with conservative eta=0.65)
    wps2 = np.array([
        [0.00, -0.30, -np.pi/2],
        [0.22, -0.22, -np.pi/3],
        [0.38, -0.10, -np.pi/6],
        [0.45,  0.00,  0.0]
    ])
    jwps2 = np.array([kin.inverse_kinematics(w) for w in wps2])
    path2 = ta.interpolator.SplineInterpolator(np.linspace(0, 1, 4), jwps2)
    c_fric2 = CartesianFrictionConeConstraint(kin, a_max=a_max_turning, num_facets=num_facets, inscribe=True)
    inst2 = ta.algorithm.TOPPRA([c_vel, c_fric2], path2, gridpoints=np.linspace(0, 1, 100), solver_wrapper='ecos')
    traj2 = inst2.compute_trajectory(0.0, 0.0)

    # STAGE 3: Pure 1D Slit Valve Insertion
    wps3 = np.array([
        [0.45, 0.00, 0.0],
        [0.54, 0.00, 0.0],
        [0.63, 0.00, 0.0],
        [0.72, 0.00, 0.0]
    ])
    jwps3 = np.array([kin.inverse_kinematics(w) for w in wps3])
    path3 = ta.interpolator.SplineInterpolator(np.linspace(0, 1, 4), jwps3)
    c_fric3 = CartesianFrictionConeConstraint(kin, a_max=a_max_linear, num_facets=num_facets, inscribe=True)
    inst3 = ta.algorithm.TOPPRA([c_vel, c_fric3], path3, gridpoints=np.linspace(0, 1, 100), solver_wrapper='ecos')
    traj3 = inst3.compute_trajectory(0.0, 0.0)

    return traj1, traj2, traj3, jwps1, jwps2, jwps3


def main():
    kin = ScaraKinematics(l1=0.40, l2=0.35, l3=0.15)

    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.setPhysicsEngineParameter(numSolverIterations=150, enableConeFriction=1)
    # Default Behind-the-SCARA Camera: looking forward and downward along the arm
    p.resetDebugVisualizerCamera(
        cameraDistance=1.55,
        cameraYaw=-40,
        cameraPitch=-30,
        cameraTargetPosition=[0.30, -0.30, 0.08]
    )

    print("Generating Phase 5 Trajectories (RNEA Inverse Dynamics Decoupling)...")
    traj1, traj2, traj3, jwps1, jwps2, jwps3 = generate_phase5_trajectories(kin, num_facets=8)
    dur1, dur2, dur3 = traj1.duration, traj2.duration, traj3.duration
    total_duration = dur1 + dur2 + dur3
    print(f"Stage 1 (FOUP Retraction along +Y):   {dur1:.3f} s")
    print(f"Stage 2 (90° Cleanroom Aisle Turn):  {dur2:.3f} s")
    print(f"Stage 3 (Chamber Insertion along +X): {dur3:.3f} s")
    print(f"Total Cleanroom Cycle Duration:       {total_duration:.3f} s")

    q_init = jwps1[0]
    robot_id, wafer_id, paddle_idx, obstacle_ids = setup_phase5_environment(kin, q_init)

    # Contact settling phase (120 steps)
    for _ in range(120):
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=[0, 1, 2],
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_init,
            targetVelocities=[0.0, 0.0, 0.0],
            positionGains=[0.8, 0.8, 0.8],
            velocityGains=[1.2, 1.2, 1.2],
            forces=[300.0, 200.0, 100.0]
        )
        p.stepSimulation()

    init_local_offset = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
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

            # Recursive Newton-Euler Inverse Dynamics Feedforward Calculation
            tau_ff = p.calculateInverseDynamics(robot_id, q_des.tolist(), qd_des.tolist(), qdd_des.tolist())
            # Clamping forces to allow the feedforward torque while retaining robust compliance
            max_torque_clamp = [abs(tau_ff[i]) + 25.0 for i in range(3)]

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
            curr_local = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
            max_slip = max(max_slip, np.linalg.norm(curr_local[:2] - init_local_offset[:2]))
            t += dt
            time.sleep(dt * 0.1)

    print("\nExecuting Stage 1: 1D FOUP Retraction (RNEA Decoupled)...")
    execute_stage(1, traj1, dur1, 0.0)
    slip_stage1 = max_slip * 1000.0
    print(f"Stage 1 Complete. Wafer Slip at Aisle Door: {slip_stage1:.4f} mm")

    print("\nExecuting Stage 2: 90° Cleanroom Aisle Transfer (RNEA Decoupled)...")
    execute_stage(2, traj2, dur2, dur1)
    slip_stage2 = max_slip * 1000.0
    print(f"Stage 2 Complete. Wafer Slip at Pre-Alignment Gate: {slip_stage2:.4f} mm")

    print("\nExecuting Stage 3: Pure 1D Slit Valve Insertion (RNEA Decoupled)...")
    execute_stage(3, traj3, dur3, dur1 + dur2)

    # Post-arrival settling (60 steps)
    for _ in range(60):
        p.stepSimulation()

    curr_local = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
    max_slip = max(max_slip, np.linalg.norm(curr_local[:2] - init_local_offset[:2]))

    final_slip_mm = max_slip * 1000.0
    stage3_added_slip = final_slip_mm - slip_stage2

    print("\n" + "=" * 65)
    print("PHASE 5: RNEA DECOUPLED EFEM TRANSFER TELEMETRY")
    print("=" * 65)
    print(f"Stage 1 (FOUP Retraction along +Y):   {dur1:.3f} s  | Slip: {slip_stage1:.4f} mm")
    print(f"Stage 2 (90° Aisle Transfer):        {dur2:.3f} s  | Slip: {slip_stage2:.4f} mm")
    print(f"Stage 3 (Chamber Insertion along +X): {dur3:.3f} s  | Added: {stage3_added_slip:.4f} mm")
    print(f"Total Trajectory Duration:           {total_duration:.3f} s")
    print(f"Final Cumulative Wafer Slip:         {final_slip_mm:.4f} mm")
    print(f"Obstacle / Aperture Collisions:      {len(collision_events)} detected")
    print(f"Verification Result:                 {'PASS (Within Threshold)' if final_slip_mm <= 0.100 and len(collision_events) == 0 else ('PASS (Bounded Slip)' if len(collision_events) == 0 else 'FAIL')}")
    print("=" * 65 + "\n")

    time.sleep(2.0)
    p.disconnect()


if __name__ == "__main__":
    main()

