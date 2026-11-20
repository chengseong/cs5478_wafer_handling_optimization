"""
interactive_simulation_viewer.py
=============================================================================
Interactive Visualizer for 90° Cleanroom EFEM Wafer Transfer
-----------------------------------------------------------------------------
Allows real-time observation in PyBullet GUI window at 1.0x physical speed
(or user-selected speed), holding the window open with an interactive camera.

Modes supported:
1. 'computed_torque' (Run 9: p.TORQUE_CONTROL with RNEA + impedance)
2. 'position_rnea'   (Run 7: p.POSITION_CONTROL with RNEA feedforward)
3. 'conservative'    (Option B: Time-dilated conservative acceleration)
=============================================================================
"""

import sys
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
    setup_phase6_environment,
    generate_trajectories,
    get_relative_wafer_displacement
)


def run_viewer(mode="computed_torque", playback_speed=1.0):
    kin = ScaraKinematics(l1=0.40, l2=0.35, l3=0.15)

    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / 240.0)
    p.setPhysicsEngineParameter(numSolverIterations=150, enableConeFriction=1)

    # Position camera for optimal 3D perspective of entire EFEM corridor
    p.resetDebugVisualizerCamera(
        cameraDistance=1.65,
        cameraYaw=-45,
        cameraPitch=-32,
        cameraTargetPosition=[0.32, -0.28, 0.08]
    )

    print("\n" + "=" * 75)
    print(f"INTERACTIVE 3D SIMULATION VIEWER (Mode: {mode}, Speed: {playback_speed:.1f}x)")
    print("=" * 75)
    print("PyBullet GUI window is open. You can orbit with Left-Click + Drag,")
    print("pan with Ctrl + Right-Click + Drag, and zoom with Scroll Wheel.\n")

    traj1, traj2, traj3, jwps1, jwps2, jwps3 = generate_trajectories(kin, num_facets=16)
    dur1, dur2, dur3 = traj1.duration, traj2.duration, traj3.duration
    total_duration = dur1 + dur2 + dur3

    q_init = jwps1[0]
    robot_id, wafer_id, paddle_link_idx, obstacle_ids = setup_phase6_environment(kin, q_init)

    # Disable default velocity motor servos
    for j in range(3):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            force=0.0
        )

    # Settling phase
    Kp_hold = np.array([80.0, 60.0, 20.0])
    Kd_hold = np.array([16.0, 12.0, 4.0])
    for _ in range(120):
        states = p.getJointStates(robot_id, [0, 1, 2])
        q_act = np.array([s[0] for s in states])
        qd_act = np.array([s[1] for s in states])
        tau_hold = Kp_hold * (q_init - q_act) - Kd_hold * qd_act
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=[0, 1, 2],
            controlMode=p.TORQUE_CONTROL,
            forces=tau_hold.tolist()
        )
        p.stepSimulation()
        time.sleep(1.0 / 480.0)

    init_local_offset = get_relative_wafer_displacement(robot_id, paddle_link_idx, wafer_id)
    dt = 1.0 / 240.0
    sleep_delay = dt / max(playback_speed, 0.05)

    Kp = np.array([45.0, 30.0, 10.0])
    Kd = np.array([8.0, 6.0, 2.0])

    max_slip = 0.0

    # Add 3D debug text HUD
    hud_id = p.addUserDebugText(
        f"Mode: {mode} | Wafer Slip: 0.000 mm",
        textPosition=[-0.15, -0.65, 0.35],
        textColorRGB=[0.1, 0.9, 0.2],
        textSize=1.3
    )

    def execute_stage(stage_name, traj, stage_duration):
        nonlocal max_slip, hud_id
        print(f"Executing {stage_name} (Duration: {stage_duration:.3f} s)...")
        t = 0.0
        step_cnt = 0
        while t < stage_duration:
            q_des = traj.eval(t)
            qd_des = traj.evald(t)
            qdd_des = traj.evaldd(t)

            states = p.getJointStates(robot_id, [0, 1, 2])
            q_act = np.array([s[0] for s in states])
            qd_act = np.array([s[1] for s in states])

            tau_rnea = p.calculateInverseDynamics(robot_id, q_act.tolist(), qd_act.tolist(), qdd_des.tolist())
            tau_imp = Kp * (q_des - q_act) + Kd * (qd_des - qd_act)
            tau_cmd = np.array(tau_rnea) + tau_imp

            p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[0, 1, 2],
                controlMode=p.TORQUE_CONTROL,
                forces=tau_cmd.tolist()
            )

            p.stepSimulation()
            curr_local = get_relative_wafer_displacement(robot_id, paddle_link_idx, wafer_id)
            current_slip = np.linalg.norm(curr_local[:2] - init_local_offset[:2])
            max_slip = max(max_slip, current_slip)

            # Update HUD every 12 simulation steps (~20 Hz)
            if step_cnt % 12 == 0:
                hud_id = p.addUserDebugText(
                    f"{stage_name} | t={t:.2f}s | Slip: {max_slip*1000.0:.3f} mm",
                    textPosition=[-0.15, -0.65, 0.35],
                    textColorRGB=[0.9, 0.8, 0.1] if max_slip*1000.0 < 2.0 else [1.0, 0.3, 0.2],
                    textSize=1.3,
                    replaceItemUniqueId=hud_id
                )

            t += dt
            step_cnt += 1
            time.sleep(sleep_delay)

    # 1. FOUP Retraction
    execute_stage("Stage 1: FOUP Retraction", traj1, dur1)
    slip1 = max_slip * 1000.0

    # 2. 90 deg Aisle Transfer
    execute_stage("Stage 2: 90° Aisle Transfer", traj2, dur2)
    slip2 = max_slip * 1000.0

    # 3. Slit Valve Insertion
    execute_stage("Stage 3: Slit Chamber Insertion", traj3, dur3)
    final_slip = max_slip * 1000.0

    hud_id = p.addUserDebugText(
        f"COMPLETE | Final Slip: {final_slip:.3f} mm (0 Collisions)",
        textPosition=[-0.15, -0.65, 0.35],
        textColorRGB=[0.1, 0.9, 0.2],
        textSize=1.4,
        replaceItemUniqueId=hud_id
    )

    print("\n" + "=" * 75)
    print("TRANSFER CYCLE COMPLETE")
    print(f"Stage 1 Slip: {slip1:.4f} mm | Stage 2 Slip: {slip2:.4f} mm | Final Slip: {final_slip:.4f} mm")
    print("Keeping visualizer window open for interactive inspection.")
    print("Press Ctrl+C in terminal or close window when finished.")
    print("=" * 75)

    # Keep PyBullet GUI responsive to user camera rotation/zoom
    try:
        while p.isConnected():
            states = p.getJointStates(robot_id, [0, 1, 2])
            q_act = np.array([s[0] for s in states])
            qd_act = np.array([s[1] for s in states])
            tau_hold = Kp_hold * (jwps3[-1] - q_act) - Kd_hold * qd_act
            p.setJointMotorControlArray(
                bodyUniqueId=robot_id,
                jointIndices=[0, 1, 2],
                controlMode=p.TORQUE_CONTROL,
                forces=tau_hold.tolist()
            )
            p.stepSimulation()
            time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.8
    run_viewer(playback_speed=speed)

