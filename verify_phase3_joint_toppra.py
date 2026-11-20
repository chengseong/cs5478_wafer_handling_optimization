import time
import numpy as np
import pybullet as p
import pybullet_data
import toppra as ta
from scara_kinematics import ScaraKinematics
from cartesian_friction_constraint import CartesianFrictionConeConstraint
from verify_wafer_transfer_scara import setup_environment


def get_relative_wafer_displacement(robot_id, paddle_link_idx, wafer_id):
    """
    Computes link-relative wafer displacement with computeForwardKinematics=1
    to prevent 1-timestep asynchronous link state phase lag in PyBullet.
    """
    link_state = p.getLinkState(robot_id, paddle_link_idx, computeForwardKinematics=1)
    link_pos = np.array(link_state[0])
    link_orn = link_state[1]

    wafer_pos, _ = p.getBasePositionAndOrientation(wafer_id)
    wafer_pos = np.array(wafer_pos)

    # Express wafer center in the moving coordinate frame of the paddle link
    inv_link_pos, inv_link_orn = p.invertTransform(link_pos, link_orn)
    local_wafer_pos, _ = p.multiplyTransforms(inv_link_pos, inv_link_orn, wafer_pos, [0, 0, 0, 1])
    return np.array(local_wafer_pos)


def generate_phase3_joint_trajectory(kin, cartesian_wps, mu=0.25, g=9.81, safety_factor=0.85, num_facets=8):
    """
    Generates a time-optimal joint-space trajectory subject to the true 2-norm Cartesian
    friction cone constraint: ||J(q)*q_ddot + J_dot(q, q_dot)*q_dot|| <= eta * mu * g
    linearized via an m-facet polygon.
    """
    # 1. Map Cartesian waypoints to Joint Space offline
    joint_wps = np.array([kin.inverse_kinematics(w) for w in cartesian_wps])
    ss_wps = np.linspace(0, 1, len(joint_wps))

    # 2. Build joint-space spline path
    path = ta.interpolator.SplineInterpolator(ss_wps, joint_wps)

    # 3. Formulate custom 2-norm friction constraint
    a_max = safety_factor * mu * g  # ~2.085 m/s^2
    c_friction = CartesianFrictionConeConstraint(kin, a_max=a_max, num_facets=num_facets, inscribe=True)
    c_vel = ta.constraint.JointVelocityConstraint([2.0, 2.0, 4.0])

    # 4. Solve time-optimal parameterization
    gridpoints = np.linspace(0, 1, 200)
    instance = ta.algorithm.TOPPRA([c_vel, c_friction], path, gridpoints=gridpoints, solver_wrapper='ecos')
    traj = instance.compute_trajectory(0.0, 0.0)

    if traj is None:
        raise RuntimeError("TOPP-RA failed to compute a feasible trajectory.")

    return traj, joint_wps


def main():
    kin = ScaraKinematics(l1=0.40, l2=0.35, l3=0.15)

    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.setPhysicsEngineParameter(numSolverIterations=150, enableConeFriction=1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.1,
        cameraYaw=50,
        cameraPitch=-35,
        cameraTargetPosition=[0.55, -0.10, 0.10]
    )

    cartesian_wps = np.array([
        [0.45, -0.20, 0.0],
        [0.52, -0.10, 0.0],
        [0.60,  0.00, 0.0],
        [0.72,  0.00, 0.0]
    ])

    print("Generating Phase 3 Joint-Space TOPP-RA trajectory (8-facet friction constraint)...")
    traj, joint_wps = generate_phase3_joint_trajectory(kin, cartesian_wps, num_facets=8)
    duration = traj.duration
    print(f"Optimal Phase 3 Trajectory Duration: {duration:.3f} s")

    q_init = joint_wps[0]
    robot_id, wafer_id, paddle_idx = setup_environment(kin, q_init)

    # Filter non-physical URDF link collisions (forearm cylinder protrusion into wafer rear manifold)
    p.setCollisionFilterPair(robot_id, wafer_id, 0, -1, enableCollision=0)
    p.setCollisionFilterPair(robot_id, wafer_id, 1, -1, enableCollision=0)

    # Contact settling phase
    for _ in range(120):
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=[0, 1, 2],
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_init,
            targetVelocities=[0.0, 0.0, 0.0],
            positionGains=[1.0, 1.0, 1.0],
            velocityGains=[1.0, 1.0, 1.0],
            forces=[300.0, 200.0, 100.0]
        )
        p.stepSimulation()

    init_local_offset = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
    dt = 1.0 / 240.0
    t = 0.0
    max_slip = 0.0
    collision_events = []

    print("Executing Phase 3 transfer simulation...")
    while t < duration:
        q_des = traj.eval(t)
        qd_des = traj.evald(t)

        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=[0, 1, 2],
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_des,
            targetVelocities=qd_des,
            positionGains=[1.0, 1.0, 1.0],
            velocityGains=[1.0, 1.0, 1.0],
            forces=[300.0, 200.0, 100.0]
        )
        p.stepSimulation()

        # Real-time collision monitoring: detect wall impact or unintended contact
        cps = p.getContactPoints()
        for c in cps:
            # Check if collision involves slit valve aperture walls (body 2 or 3)
            if c[1] in [2, 3] or c[2] in [2, 3]:
                wall_body = c[1] if c[1] in [2, 3] else c[2]
                robot_link = c[3] if c[1] == robot_id else (c[4] if c[2] == robot_id else -1)
                link_name = ["Shoulder (Link 1)", "Forearm (Link 2)", "Paddle"][robot_link] if 0 <= robot_link <= 2 else "Unknown"
                wall_name = "Upper Plate" if wall_body == 2 else "Lower Plate"
                normal_force = c[9]
                if not any(e["time"] == round(t, 3) for e in collision_events):
                    collision_events.append({
                        "time": round(t, 3),
                        "link": link_name,
                        "wall": wall_name,
                        "force": normal_force,
                        "slip_prior": max_slip * 1000.0
                    })
                    print(f"  [COLLISION DETECTED at t={t:.3f}s] Robot {link_name} struck {wall_name}! "
                          f"Normal Force: {normal_force:.2f} N | Slip before impact: {max_slip * 1000.0:.3f} mm")

        current_local_offset = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
        planar_slip = np.linalg.norm(current_local_offset[:2] - init_local_offset[:2])
        max_slip = max(max_slip, planar_slip)

        t += dt
        time.sleep(dt * 0.2)

    # Post-arrival settling
    for _ in range(60):
        p.stepSimulation()

    slip_mm = max_slip * 1000.0
    passed = max_slip <= 0.0001 and len(collision_events) == 0  # <= 0.100 mm and collision-free

    print("\n" + "=" * 55)
    print("PHASE 3: JOINT-SPACE TOPP-RA TELEMETRY (8-FACET)")
    print("=" * 55)
    print(f"Trajectory Duration:       {duration:.3f} s")
    print(f"Max Measured Wafer Slip:   {slip_mm:.4f} mm")
    print(f"Allowable Slip Threshold:  0.1000 mm")
    print(f"Aperture Wall Collisions:  {len(collision_events)} detected")
    if collision_events:
        first_c = collision_events[0]
        print(f"  -> First collision at t={first_c['time']:.3f}s: {first_c['link']} struck {first_c['wall']} (Slip prior: {first_c['slip_prior']:.3f} mm)")
    print(f"Verification Result:       {'PASS' if passed else 'FAIL'}")
    print("=" * 55 + "\n")

    p.disconnect()


if __name__ == "__main__":
    main()

