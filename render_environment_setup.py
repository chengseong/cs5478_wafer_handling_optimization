"""
Enhanced synthetic visualization generator for the final 90° Cleanroom EFEM setup.
Generates:
1. Annotated Multi-Panel 3D Simulation Overview (Isometric, Floorplan, Pre-alignment, Slit Insertion)
2. Trajectory Waypoints & Motion Vectors Kinematic Map
"""

import os
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

from scara_kinematics import ScaraKinematics
from verify_phase7_lcp_stabilized_transfer import setup_phase7_environment, generate_phase7_trajectories

def render_camera_view(camera_pos, target_pos, up_vec=[0, 0, 1], fov=55, width=1280, height=720):
    view_mat = p.computeViewMatrix(cameraEyePosition=camera_pos,
                                   cameraTargetPosition=target_pos,
                                   cameraUpVector=up_vec)
    proj_mat = p.computeProjectionMatrixFOV(fov=fov, aspect=float(width)/height, nearVal=0.1, farVal=5.0)
    img_arr = p.getCameraImage(width=width, height=height, viewMatrix=view_mat, projectionMatrix=proj_mat,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba = np.reshape(img_arr[2], (height, width, 4))
    return rgba

def generate_kinematic_map(kin, traj1, traj2, traj3, output_path):
    fig, ax = plt.subplots(figsize=(8, 7), dpi=300)
    
    # Floorplan obstacles
    # Robot Base at (0, 0)
    base_circle = plt.Circle((0, 0), 0.10, color='dimgray', alpha=0.9, label='SCARA Base ($R=0.10$ m)')
    ax.add_patch(base_circle)
    
    # Reachable workspace circle (L1 + L2 + L3 = 0.90m)
    reach_circle = plt.Circle((0, 0), 0.90, color='lightgray', fill=False, linestyle='--', label='Max Reach ($0.90$ m)')
    ax.add_patch(reach_circle)
    
    # FOUP Enclosure box at (0.0, -0.65)
    foup_rect = patches.Rectangle((-0.20, -0.85), 0.40, 0.36, linewidth=1.8, edgecolor='navy', facecolor='dodgerblue', alpha=0.35, label='FOUP Enclosure (SEMI Load Port)')
    ax.add_patch(foup_rect)
    ax.plot([-0.20, 0.20], [-0.49, -0.49], 'r--', linewidth=2.0, label='FOUP Front Door (+Y Open Aisle)')
    
    # Process Chamber Slit Valve Aperture at X = 0.60, Y in [-0.175, 0.175]
    ax.plot([0.60, 0.60], [-0.175, 0.175], 'purple', linewidth=4.0, label='Slit Valve Aperture (350 mm $\\times$ 50 mm)')
    chamber_rect = patches.Rectangle((0.60, -0.22), 0.30, 0.44, linewidth=1.5, edgecolor='purple', facecolor='mediumorchid', alpha=0.2, label='Process Vacuum Chamber')
    ax.add_patch(chamber_rect)
    
    # Trajectory 1: Retraction along +Y
    t1_samples = np.linspace(0, traj1.duration, 50)
    pts1 = np.array([kin.forward_kinematics(traj1.eval(t))[:2] for t in t1_samples])
    ax.plot(pts1[:, 0], pts1[:, 1], 'tab:blue', linewidth=3.0, label='Stage 1: Straight FOUP Retraction ($+Y$)')
    
    # Trajectory 2: 90° Open Aisle Transfer
    t2_samples = np.linspace(0, traj2.duration, 80)
    pts2 = np.array([kin.forward_kinematics(traj2.eval(t))[:2] for t in t2_samples])
    ax.plot(pts2[:, 0], pts2[:, 1], 'tab:green', linewidth=3.0, label='Stage 2: 90° Open Aisle Transfer')
    
    # Trajectory 3: Straight Slit Insertion along +X
    t3_samples = np.linspace(0, traj3.duration, 50)
    pts3 = np.array([kin.forward_kinematics(traj3.eval(t))[:2] for t in t3_samples])
    ax.plot(pts3[:, 0], pts3[:, 1], 'tab:red', linewidth=3.0, label='Stage 3: Slit Insertion into Chamber ($+X$)')
    
    # Key waypoints markers
    w_pick = pts1[0]
    w_door = pts1[-1]
    w_gate = pts2[-1]
    w_target = pts3[-1]
    
    ax.plot(w_pick[0], w_pick[1], 'o', color='darkblue', markersize=8)
    ax.text(w_pick[0]+0.02, w_pick[1]-0.04, 'Pick: [0.0, -0.65]\n($\\phi = -90^\\circ$)', fontsize=9, fontweight='bold', color='darkblue')
    
    ax.plot(w_door[0], w_door[1], 's', color='darkgreen', markersize=7)
    ax.text(w_door[0]+0.02, w_door[1]-0.03, 'Aisle Door: [0.0, -0.30]', fontsize=8.5, color='darkgreen')
    
    ax.plot(w_gate[0], w_gate[1], '^', color='darkorange', markersize=8)
    ax.text(w_gate[0]-0.05, w_gate[1]-0.07, 'Pre-Alignment Gate:\n[0.45, 0.0] ($\\phi \\equiv 0^\\circ$)', fontsize=8.5, color='saddlebrown')
    
    ax.plot(w_target[0], w_target[1], '*', color='crimson', markersize=12)
    ax.text(w_target[0]-0.08, w_target[1]+0.05, 'Terminal Target:\n[0.705, 0.0]', fontsize=9, fontweight='bold', color='crimson')
    
    # Set plot boundaries and grid
    ax.set_xlim(-0.35, 0.95)
    ax.set_ylim(-0.95, 0.35)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.set_xlabel('Global X Coordinate (meters)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Global Y Coordinate (meters)', fontsize=11, fontweight='bold')
    ax.set_title('SEMI-Standard Cleanroom EFEM Layout & Kinematic Transfer Path', fontsize=12, fontweight='bold', pad=10)
    ax.legend(loc='upper left', fontsize=8.5, framealpha=0.92)
    
    plt.tight_layout()
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()

def main():
    output_dir = r"C:\Users\Cheng\.gemini\antigravity\brain\fd3f5e3c-c6d7-4f9a-8e07-974b2000048f"
    os.makedirs(output_dir, exist_ok=True)
    
    kin = ScaraKinematics(l1=0.40, l2=0.35, l3=0.15)
    
    # 1. Generate 2D SEMI Floorplan & Trajectory Map
    traj1, traj2, traj3, jwps1, jwps2, jwps3 = generate_phase7_trajectories(kin)
    map_path = os.path.join(output_dir, "efem_kinematic_map.png")
    generate_kinematic_map(kin, traj1, traj2, traj3, map_path)

    # Save copies to repository root, overleaf_package, and powerpoint_assets
    import shutil
    root_map = "efem_kinematic_map.png"
    overleaf_map = os.path.join("overleaf_package", "figures", "fig1_efem_kinematic_map.png")
    pptx_map = os.path.join("powerpoint_assets", "slide1_efem_kinematic_map.png")
    for dst in [root_map, overleaf_map, pptx_map]:
        if os.path.exists(os.path.dirname(dst)) or not os.path.dirname(dst):
            shutil.copyfile(map_path, dst)
    
    # 2. Render 3D PyBullet Views
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Add a soft visual ground grid plane in Direct mode
    ground_vis = p.createVisualShape(p.GEOM_PLANE, rgbaColor=[0.94, 0.94, 0.96, 1.0])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=ground_vis, basePosition=[0, 0, 0])
    
    q_init = jwps1[0]
    robot_id, wafer_id, paddle_idx, obstacle_ids = setup_phase7_environment(kin, q_init)
    
    # DEFAULT BEHIND-SCARA CAMERA CONFIGURATION
    cam_eye_behind = [-0.55, 0.40, 0.75]
    cam_target_behind = [0.35, -0.30, 0.08]

    # Panel 1: Pick Pose inside FOUP (from behind SCARA base)
    img_pick = render_camera_view(camera_pos=cam_eye_behind, target_pos=cam_target_behind, fov=58)

    # Panel 2: FOUP Retraction to Aisle Door
    q_door = jwps1[-1]
    for i in range(3):
        p.resetJointState(robot_id, i, q_door[i])
    fk_door = kin.forward_kinematics(q_door)
    p.resetBasePositionAndOrientation(wafer_id, [fk_door[0], fk_door[1], 0.092], p.getQuaternionFromEuler([0, 0, fk_door[2]]))
    img_door = render_camera_view(camera_pos=cam_eye_behind, target_pos=cam_target_behind, fov=58)

    # Panel 3: Pre-Alignment Gate Aisle View
    q_mid = jwps2[-1]
    for i in range(3):
        p.resetJointState(robot_id, i, q_mid[i])
    fk_mid = kin.forward_kinematics(q_mid)
    p.resetBasePositionAndOrientation(wafer_id, [fk_mid[0], fk_mid[1], 0.092], p.getQuaternionFromEuler([0, 0, fk_mid[2]]))
    img_aisle = render_camera_view(camera_pos=cam_eye_behind, target_pos=cam_target_behind, fov=58)

    # Panel 4: Slit Valve Chamber Insertion Terminal View
    q_end = jwps3[-1]
    for i in range(3):
        p.resetJointState(robot_id, i, q_end[i])
    fk_end = kin.forward_kinematics(q_end)
    p.resetBasePositionAndOrientation(wafer_id, [fk_end[0], fk_end[1], 0.092], p.getQuaternionFromEuler([0, 0, fk_end[2]]))
    img_chamber = render_camera_view(camera_pos=cam_eye_behind, target_pos=cam_target_behind, fov=58)
    p.disconnect()

    # 3. Assemble High-Resolution Multi-Panel Figure for Slide Deck
    fig, axes = plt.subplots(2, 2, figsize=(16, 10), dpi=250)

    axes[0, 0].imshow(img_pick)
    axes[0, 0].set_title("(a) Pick State inside FOUP Enclosure\nWafer at [0.0, -0.65], Yaw phi = -90°", fontsize=11.5, fontweight='bold', pad=8)
    axes[0, 0].axis('off')

    axes[0, 1].imshow(img_door)
    axes[0, 1].set_title("(b) Stage 1: Linear Extraction to Aisle Door\nStraight +Y Motion to [0.0, -0.30] (0 Collisions)", fontsize=11.5, fontweight='bold', pad=8)
    axes[0, 1].axis('off')

    axes[1, 0].imshow(img_aisle)
    axes[1, 0].set_title("(c) Stage 2: 90° Cleanroom Aisle Transfer\nPre-Alignment Gate at [0.45, 0.0], Yaw Decoupled to phi = 0°", fontsize=11.5, fontweight='bold', pad=8)
    axes[1, 0].axis('off')

    axes[1, 1].imshow(img_chamber)
    axes[1, 1].set_title("(d) Stage 3: Slit Valve Chamber Insertion\nStraight +X Stroke to Terminal Chuck [0.705, 0.0]", fontsize=11.5, fontweight='bold', pad=8)
    axes[1, 1].axis('off')
    plt.tight_layout()
    overview_path = os.path.join(output_dir, "final_environment_setup_detailed.png")
    plt.savefig(overview_path, bbox_inches='tight')
    plt.close()
    
    print(f"Generated 2D Kinematic Map: {map_path}")
    print(f"Generated 3D Multi-panel:   {overview_path}")

if __name__ == "__main__":
    main()

