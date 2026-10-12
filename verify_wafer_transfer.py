import time
import numpy as np
import pybullet as p
import pybullet_data
import toppra as ta
from toppra import constraint, interpolator

# ==========================================
# 1. Trajectory Optimization via TOPP-RA
# ==========================================
def generate_slip_constrained_trajectory(mu=0.25, g=9.81, safety_factor=0.85):
    """
    Computes a time-optimal trajectory with an engineering safety margin:
    ||a|| <= eta * mu * g (where eta = 0.85).
    """
    waypoints = np.array([
        [0.00, 0.0],
        [0.10, 0.0],
        [0.30, 0.0],
        [0.50, 0.0],
        [0.60, 0.0]
    ])
    
    ss_waypoints = np.linspace(0, 1, len(waypoints))
    path = interpolator.SplineInterpolator(ss_waypoints, waypoints, bc_type='clamped')

    a_max_friction = safety_factor * mu * g  # ~2.085 m/s^2
    v_max = 1.2                             # m/s

    pc_vel = constraint.JointVelocityConstraint(np.array([v_max, v_max]))
    pc_acc = constraint.JointAccelerationConstraint(np.array([a_max_friction, a_max_friction]))

    gridpoints = np.linspace(0, 1, 150)
    instance = ta.algorithm.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints, solver_wrapper='ecos')
    trajectory = instance.compute_trajectory(0.0, 0.0)

    if trajectory is None:
        raise RuntimeError("TOPP-RA failed to compute a feasible trajectory.")

    print(f"Trajectory solved. Duration: {trajectory.duration:.3f} s")
    print(f"Acceleration limit (eta={safety_factor}): {a_max_friction:.3f} m/s^2")
    return trajectory


# ==========================================
# 2. PyBullet Simulation Setup
# ==========================================
def setup_sim():
    client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.setPhysicsEngineParameter(
        numSolverIterations=150,
        enableConeFriction=1
    )
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=0.85, cameraYaw=40, cameraPitch=-30, cameraTargetPosition=[0.3, 0.0, 0.04]
    )
    return client

def build_primitives():
    # Paddle: 20mm thick collision block (half-extent Z = 0.01)
    # Center at Z = 0.03, so top surface is exactly at Z = 0.040
    pad_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.16, 0.10, 0.01])
    pad_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.16, 0.10, 0.003], rgbaColor=[0.85, 0.85, 0.85, 1.0])
    paddle_id = p.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=pad_col,
        baseVisualShapeIndex=pad_vis,
        basePosition=[0.0, 0.0, 0.03]
    )
    p.changeDynamics(paddle_id, -1, lateralFriction=1.0, restitution=0.0)

    # Rigid kinematic anchor constraint (erp=0.9 eliminates spring lag)
    paddle_cid = p.createConstraint(
        parentBodyUniqueId=paddle_id,
        parentLinkIndex=-1,
        childBodyUniqueId=-1,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0.0, 0.0, 0.03]
    )
    p.changeConstraint(paddle_cid, erp=0.9)

    # Wafer: 4mm thick cylinder, rests flush at Z = 0.042 (bottom at Z = 0.040)
    w_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.15, height=0.004)
    w_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.15, length=0.002, rgbaColor=[0.2, 0.25, 0.35, 1.0])
    wafer_id = p.createMultiBody(
        baseMass=0.128,
        baseCollisionShapeIndex=w_col,
        baseVisualShapeIndex=w_vis,
        basePosition=[0.0, 0.0, 0.042]
    )
    p.changeDynamics(wafer_id, -1, lateralFriction=0.25, restitution=0.0)

    # Slit Valve Aperture: 350mm wide x 50mm high
    wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.25, 0.05])
    wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.25, 0.05], rgbaColor=[0.5, 0.5, 0.6, 0.4])
    # Top plate (lip at Z = 0.065)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col,
                      baseVisualShapeIndex=wall_vis, basePosition=[0.40, 0.0, 0.115])
    # Bottom plate (lip at Z = 0.015)
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_col,
                      baseVisualShapeIndex=wall_vis, basePosition=[0.40, 0.0, -0.035])

    return paddle_id, wafer_id, paddle_cid


# ==========================================
# 3. Execution & Slip Telemetry
# ==========================================
def main():
    traj = generate_slip_constrained_trajectory(mu=0.25, g=9.81, safety_factor=0.85)
    setup_sim()
    paddle, wafer, paddle_cid = build_primitives()

    # Settle contact in gravity
    for _ in range(120):
        p.stepSimulation()

    pad_pos_0, _ = p.getBasePositionAndOrientation(paddle)
    waf_pos_0, _ = p.getBasePositionAndOrientation(wafer)
    initial_offset = np.array(waf_pos_0[:2]) - np.array(pad_pos_0[:2])

    dt = 1.0 / 240.0
    total_steps = int(traj.duration / dt)
    max_relative_slip = 0.0

    print("Executing transfer trajectory...")
    for step in range(total_steps):
        t_curr = step * dt
        xy_target = traj(t_curr)

        # Update anchor target position rigidly
        target_pos = [float(xy_target[0]), float(xy_target[1]), 0.03]
        p.changeConstraint(paddle_cid, target_pos, maxForce=200000, erp=0.9)

        p.stepSimulation()
        time.sleep(dt * 0.5)

        # Track planar relative offset
        pad_pos, _ = p.getBasePositionAndOrientation(paddle)
        waf_pos, _ = p.getBasePositionAndOrientation(wafer)
        current_offset = np.array(waf_pos[:2]) - np.array(pad_pos[:2])
        slip_displacement = np.linalg.norm(current_offset - initial_offset)

        if slip_displacement > max_relative_slip:
            max_relative_slip = slip_displacement

    # Settle after arrival
    for _ in range(60):
        p.stepSimulation()

    print("\n================ Results ================")
    print(f"Total Trajectory Time : {traj.duration:.3f} s")
    print(f"Max Wafer Slip Offset : {max_relative_slip * 1000.0:.4f} mm")
    if max_relative_slip < 1e-3:
        print("Status                : PASS (Slip bounded below 1.0 mm)")
    else:
        print("Status                : FAIL (Slip constraint violated)")
    print("=========================================\n")

    time.sleep(2.0)
    p.disconnect()

if __name__ == "__main__":
    main()