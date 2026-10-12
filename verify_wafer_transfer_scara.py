import time
import numpy as np
import pybullet as p
import pybullet_data
import toppra as ta
from toppra import constraint, interpolator
from scara_kinematics import ScaraKinematics


def compute_full_jacobian(kin, q):
    """
    Computes the 3x3 operational space Jacobian mapping joint rates
    [th1_dot, th2_dot, th3_dot] -> [x_dot, y_dot, phi_dot].
    """
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


def generate_cartesian_slip_bounded_trajectory(cartesian_wps, mu=0.25, g=9.81, safety_factor=0.85):
    """
    Generates a time-optimal Cartesian path profile where ||a_tip|| <= eta * mu * g
    is strictly guaranteed across all spatial axes.
    """
    ss_wps = np.linspace(0, 1, len(cartesian_wps))
    path = interpolator.SplineInterpolator(ss_wps, cartesian_wps, bc_type='clamped')

    # Component-wise projection enforcing the 2-norm limit: sqrt(ax^2 + ay^2) <= a_max
    a_max_friction = safety_factor * mu * g  # 0.85 * 0.25 * 9.81 ≈ 2.085 m/s²
    a_max_comp = a_max_friction / np.sqrt(2.0)  # ~1.474 m/s² per planar axis

    v_max = 1.0

    # Constraints on [X, Y, Phi]
    pc_vel = constraint.JointVelocityConstraint([v_max, v_max, 2.0])
    pc_acc = constraint.JointAccelerationConstraint([a_max_comp, a_max_comp, 4.0])

    instance = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, gridpoints=np.linspace(0, 1, 200))
    traj = instance.compute_trajectory(0.0, 0.0)
    return traj


def setup_simulation():
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.setPhysicsEngineParameter(
        numSolverIterations=150,
        enableConeFriction=1
    )
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    return client


def setup_environment(kin, q_init):
    # 1. Load articulated SCARA robot
    robot_id = p.loadURDF("scara_wafer.urdf", [0, 0, 0], useFixedBase=True)
    paddle_link_idx = 2

    # Calibrate paddle friction surface (mu_paddle = 1.0 -> effective mu = 1.0 * 0.25 = 0.25)
    p.changeDynamics(
        robot_id,
        paddle_link_idx,
        lateralFriction=1.0,
        restitution=0.0
    )

    # Initialize joint states
    for i in range(3):
        p.resetJointState(robot_id, i, q_init[i])

    # 2. Wafer primitive (300mm diameter, 128g mass, 4mm collision thickness)
    w_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.15, height=0.004)
    w_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.15, length=0.002, rgbaColor=[0.2, 0.25, 0.35, 1.0])

    init_fk = kin.forward_kinematics(q_init)
    # Paddle top surface sits at Z = 0.090m; cylinder center placed at Z = 0.092m
    wafer_id = p.createMultiBody(
        baseMass=0.128,
        baseCollisionShapeIndex=w_col,
        baseVisualShapeIndex=w_vis,
        basePosition=[init_fk[0], init_fk[1], 0.092]
    )
    p.changeDynamics(
        wafer_id,
        -1,
        lateralFriction=0.25,
        restitution=0.0
    )

    # 3. Slit valve aperture enclosure (350mm x 50mm opening at X = 0.60m)
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.25, 0.05])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.25, 0.05], rgbaColor=[0.5, 0.5, 0.6, 0.4])

    # Upper plate (Z: 0.115m to 0.215m)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col, baseVisualShapeIndex=wall_vis,
                      basePosition=[0.60, 0.0, 0.165])
    # Lower plate (Z: -0.035m to 0.065m) -> Clear aperture window: Z in [0.065m, 0.115m]
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col, baseVisualShapeIndex=wall_vis,
                      basePosition=[0.60, 0.0, 0.015])

    return robot_id, wafer_id, paddle_link_idx


def get_relative_wafer_displacement(robot_id, paddle_link_idx, wafer_id):
    link_state = p.getLinkState(robot_id, paddle_link_idx)
    link_pos = np.array(link_state[0])
    link_orn = link_state[1]

    wafer_pos, _ = p.getBasePositionAndOrientation(wafer_id)
    wafer_pos = np.array(wafer_pos)

    # Express wafer center in the moving coordinate frame of the paddle link
    inv_link_pos, inv_link_orn = p.invertTransform(link_pos, link_orn)
    local_wafer_pos, _ = p.multiplyTransforms(inv_link_pos, inv_link_orn, wafer_pos, [0, 0, 0, 1])
    return np.array(local_wafer_pos)


def main():
    kin = ScaraKinematics(l1=0.40, l2=0.35, l3=0.15)
    setup_simulation()

    # Cartesian waypoints: FOUP pick -> Chamber entry through slit valve
    cartesian_wps = np.array([
        [0.45, -0.20, 0.0],
        [0.52, -0.10, 0.0],
        [0.60,  0.00, 0.0],  # Aligned with slit valve aperture center
        [0.72,  0.00, 0.0]   # Chamber terminal position
    ])

    traj = generate_cartesian_slip_bounded_trajectory(cartesian_wps)
    duration = traj.duration

    q_init = kin.inverse_kinematics(cartesian_wps[0])
    robot_id, wafer_id, paddle_idx = setup_environment(kin, q_init)

    # Pre-motion servo lock: stabilize contact manifold without initial torque jerk
    for _ in range(120):
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=[0, 1, 2],
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_init,
            targetVelocities=[0.0, 0.0, 0.0],
            positionGains=[0.3, 0.3, 0.3],
            velocityGains=[1.0, 1.0, 1.0],
            forces=[300.0, 200.0, 100.0]
        )
        p.stepSimulation()

    init_local_offset = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)

    dt = 1.0 / 240.0
    t = 0.0
    max_slip = 0.0
    q_prev = np.copy(q_init)

    while t < duration:
        # Evaluate desired Cartesian pose and velocity
        x_des = traj.eval(t)
        xd_des = traj.evald(t)

        # Inverse Kinematics mapping to joint positions
        q_des = kin.inverse_kinematics(x_des)

        # Jacobian inversion mapping to feedforward joint velocities: q_dot = J^-1 * x_dot
        J = compute_full_jacobian(kin, q_des)
        qd_des = np.linalg.solve(J, xd_des)

        # Motor actuation with velocity feedforward
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=[0, 1, 2],
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_des,
            targetVelocities=qd_des,
            positionGains=[0.3, 0.3, 0.3],
            velocityGains=[1.0, 1.0, 1.0],
            forces=[300.0, 200.0, 100.0]
        )

        p.stepSimulation()

        # Monitor relative displacement on the paddle plane
        current_local_offset = get_relative_wafer_displacement(robot_id, paddle_idx, wafer_id)
        planar_slip = np.linalg.norm(current_local_offset[:2] - init_local_offset[:2])
        max_slip = max(max_slip, planar_slip)

        t += dt
        time.sleep(dt * 0.2)

    slip_mm = max_slip * 1000.0
    passed = max_slip <= 0.0001  # <= 0.1 mm tolerance

    print("=" * 50)
    print("SCARA WAFER TRANSFER TELEMETRY")
    print("=" * 50)
    print(f"Trajectory Duration:       {duration:.3f} s")
    print(f"Max Measured Wafer Slip:   {slip_mm:.4f} mm")
    print(f"Allowable Slip Threshold:  0.1000 mm")
    print(f"Verification Result:       {'PASS' if passed else 'FAIL'}")
    print("=" * 50)

    p.disconnect()


if __name__ == "__main__":
    main()