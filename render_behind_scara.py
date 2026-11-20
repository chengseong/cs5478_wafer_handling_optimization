"""
Renders the complete cleanroom EFEM sequence from behind the SCARA robot arm base.
Camera position: Placed at negative X, positive Y (behind the base, e.g. [-0.55, +0.45, 0.70])
looking forward and downward along the arm towards both the FOUP (-Y) and the Process Chamber (+X).
"""

import os
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
from PIL import Image

from scara_kinematics import ScaraKinematics
from verify_phase5_decoupled_efem import setup_phase5_environment, generate_phase5_trajectories

def render_camera_view(camera_pos, target_pos, up_vec=[0, 0, 1], fov=58, width=1920, height=1080):
    view_mat = p.computeViewMatrix(cameraEyePosition=camera_pos,
                                   cameraTargetPosition=target_pos,
                                   cameraUpVector=up_vec)
    proj_mat = p.computeProjectionMatrixFOV(fov=fov, aspect=float(width)/height, nearVal=0.1, farVal=6.0)
    img_arr = p.getCameraImage(width=width, height=height, viewMatrix=view_mat, projectionMatrix=proj_mat,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba = np.reshape(img_arr[2], (height, width, 4))
    return rgba

def main():
    output_dir = r"C:\Users\Cheng\.gemini\antigravity\brain\fd3f5e3c-c6d7-4f9a-8e07-974b2000048f"
    os.makedirs(output_dir, exist_ok=True)
    
    kin = ScaraKinematics(l1=0.40, l2=0.35, l3=0.15)
    traj1, traj2, traj3, jwps1, jwps2, jwps3 = generate_phase5_trajectories(kin)
    
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    ground_vis = p.createVisualShape(p.GEOM_PLANE, rgbaColor=[0.94, 0.94, 0.96, 1.0])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=-1, baseVisualShapeIndex=ground_vis, basePosition=[0, 0, 0])
    
    # 1. Pose 1: Pick Pose inside FOUP
    q_init = jwps1[0]
    robot_id, wafer_id, paddle_idx, obstacle_ids = setup_phase5_environment(kin, q_init)
    
    # "Over-the-Shoulder / Behind-the-Arm" Camera configuration:
    # Placed behind the SCARA base at [-0.55, 0.40, 0.75] looking into the EFEM cell [0.35, -0.30, 0.08]
    cam_eye_behind = [-0.55, 0.40, 0.75]
    cam_target_behind = [0.35, -0.30, 0.08]
    
    img_pick = render_camera_view(cam_eye_behind, cam_target_behind)
    Image.fromarray(img_pick).save(os.path.join(output_dir, "behind_scara_pick.png"))
    
    # 2. Pose 2: Doorway Extracted (End of Stage 1)
    q_door = jwps1[-1]
    for i in range(3):
        p.resetJointState(robot_id, i, q_door[i])
    fk_door = kin.forward_kinematics(q_door)
    p.resetBasePositionAndOrientation(wafer_id, [fk_door[0], fk_door[1], 0.092], p.getQuaternionFromEuler([0, 0, fk_door[2]]))
    img_door = render_camera_view(cam_eye_behind, cam_target_behind)
    Image.fromarray(img_door).save(os.path.join(output_dir, "behind_scara_door.png"))
    
    # 3. Pose 3: Midway Aisle Pre-Alignment Gate (End of Stage 2)
    q_mid = jwps2[-1]
    for i in range(3):
        p.resetJointState(robot_id, i, q_mid[i])
    fk_mid = kin.forward_kinematics(q_mid)
    p.resetBasePositionAndOrientation(wafer_id, [fk_mid[0], fk_mid[1], 0.092], p.getQuaternionFromEuler([0, 0, fk_mid[2]]))
    img_mid = render_camera_view(cam_eye_behind, cam_target_behind)
    Image.fromarray(img_mid).save(os.path.join(output_dir, "behind_scara_transfer.png"))
    
    # 4. Pose 4: Chamber Terminal Insertion (End of Stage 3)
    q_end = jwps3[-1]
    for i in range(3):
        p.resetJointState(robot_id, i, q_end[i])
    fk_end = kin.forward_kinematics(q_end)
    p.resetBasePositionAndOrientation(wafer_id, [fk_end[0], fk_end[1], 0.092], p.getQuaternionFromEuler([0, 0, fk_end[2]]))
    img_end = render_camera_view(cam_eye_behind, cam_target_behind)
    Image.fromarray(img_end).save(os.path.join(output_dir, "behind_scara_insertion.png"))
    
    p.disconnect()
    
    # Assemble High-Res 4-Panel Sequential Progression Figure for Slide Presentation
    fig, axes = plt.subplots(2, 2, figsize=(16, 10), dpi=250)
    
    axes[0, 0].imshow(img_pick)
    axes[0, 0].set_title("(a) Pick State inside FOUP Enclosure\nWafer at [0.0, -0.65], Yaw phi = -90°", 
                         fontsize=11.5, fontweight='bold', pad=8)
    axes[0, 0].axis('off')
    
    axes[0, 1].imshow(img_door)
    axes[0, 1].set_title("(b) Stage 1: Linear Extraction to Aisle Door\nStraight +Y Motion to [0.0, -0.30] (0 Collisions)", 
                         fontsize=11.5, fontweight='bold', pad=8)
    axes[0, 1].axis('off')
    
    axes[1, 0].imshow(img_mid)
    axes[1, 0].set_title("(c) Stage 2: 90° Cleanroom Aisle Transfer\nPre-Alignment Gate at [0.45, 0.0], Yaw Decoupled to phi = 0°", 
                         fontsize=11.5, fontweight='bold', pad=8)
    axes[1, 0].axis('off')
    
    axes[1, 1].imshow(img_end)
    axes[1, 1].set_title("(d) Stage 3: Slit Valve Chamber Insertion\nStraight +X Stroke to Terminal Chuck [0.72, 0.0]", 
                         fontsize=11.5, fontweight='bold', pad=8)
    axes[1, 1].axis('off')
    
    plt.tight_layout()
    progression_path = os.path.join(output_dir, "behind_scara_progression.png")
    plt.savefig(progression_path, bbox_inches='tight')
    plt.close()
    
    print(f"Behind-SCARA progression rendered successfully at:\n{progression_path}")

if __name__ == "__main__":
    main()

