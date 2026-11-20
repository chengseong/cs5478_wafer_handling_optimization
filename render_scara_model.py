"""
render_scara_model.py
Renders a publication-quality 2-panel figure for Section 3.1:
(a) 3D Isometric View of the 3-DOF SCARA Manipulator & Wafer Payload in PyBullet.
(b) Planar Kinematic Schematic defining joint angles (theta_1, theta_2, theta_3), link lengths (L_1, L_2, L_3), and end-effector yaw phi.
"""

import os
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image

def render_3d_scara():
    cid = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Clean light studio background
    plane_vis = p.createVisualShape(p.GEOM_PLANE, rgbaColor=[0.96, 0.97, 0.98, 1.0])
    p.createMultiBody(0, -1, plane_vis, [0, 0, -0.01])

    # Load SCARA URDF
    urdf_path = os.path.abspath("scara_wafer.urdf")
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

    # Set an articulated pose: shoulder=35 deg, elbow=50 deg, wrist to align wafer
    q = [np.deg2rad(30), np.deg2rad(50), np.deg2rad(-80)]
    for i in range(3):
        p.resetJointState(robot_id, i, q[i])

    # Add wafer on top of paddle (link index 2)
    state = p.getLinkState(robot_id, 2)
    paddle_pos = state[0]
    paddle_orn = state[1]
    wafer_pos = [paddle_pos[0], paddle_pos[1], paddle_pos[2] + 0.007]

    wafer_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.15, height=0.004)
    wafer_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.15, length=0.004,
                                   rgbaColor=[0.2, 0.45, 0.85, 0.95],
                                   specularColor=[0.8, 0.8, 0.9])
    wafer_id = p.createMultiBody(baseMass=0.128,
                                 baseCollisionShapeIndex=wafer_col,
                                 baseVisualShapeIndex=wafer_vis,
                                 basePosition=wafer_pos,
                                 baseOrientation=paddle_orn)

    # Camera positioned for high-clarity isometric angle
    cam_eye = [-0.35, -0.65, 0.70]
    cam_target = [0.28, 0.15, 0.08]
    cam_up = [0, 0, 1]

    view_mat = p.computeViewMatrix(cameraEyePosition=cam_eye,
                                   cameraTargetPosition=cam_target,
                                   cameraUpVector=cam_up)
    proj_mat = p.computeProjectionMatrixFOV(fov=48, aspect=1.33, nearVal=0.1, farVal=5.0)

    w, h = 1200, 900
    img_arr = p.getCameraImage(width=w, height=h, viewMatrix=view_mat, projectionMatrix=proj_mat,
                               renderer=p.ER_BULLET_HARDWARE_OPENGL)
    p.disconnect()

    rgba = np.reshape(img_arr[2], (h, w, 4))
    return Image.fromarray(rgba)

def render_kinematic_schematic(ax):
    # Link lengths
    L1 = 0.40
    L2 = 0.35
    L3 = 0.15

    # Angles
    th1 = np.deg2rad(32)
    th2 = np.deg2rad(48)
    th3 = np.deg2rad(-80)
    phi = th1 + th2 + th3  # 0 deg

    # Joint positions
    p0 = np.array([0.0, 0.0])
    p1 = p0 + np.array([L1 * np.cos(th1), L1 * np.sin(th1)])
    p2 = p1 + np.array([L2 * np.cos(th1 + th2), L2 * np.sin(th1 + th2)])
    p3 = p2 + np.array([L3 * np.cos(phi), L3 * np.sin(phi)])

    # Draw links
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color='#1e3a8a', linewidth=5.5, solid_capstyle='round', label='Link 1 ($L_1 = 0.40$ m)')
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='#0d9488', linewidth=4.5, solid_capstyle='round', label='Link 2 ($L_2 = 0.35$ m)')
    ax.plot([p2[0], p3[0]], [p2[1], p3[1]], color='#d97706', linewidth=3.5, solid_capstyle='round', label='Blade ($L_3 = 0.15$ m)')

    # Draw wafer at p3
    wafer_circ = patches.Circle(p3, 0.15, edgecolor='#2563eb', facecolor='#60a5fa', alpha=0.35, linewidth=2, linestyle='-', label='300 mm Wafer ($r=0.15$ m)')
    ax.add_patch(wafer_circ)
    ax.plot(p3[0], p3[1], 'o', color='#1d4ed8', markersize=6)

    # Draw joint pivots
    for pt, name in zip([p0, p1, p2], ['Shoulder Joint 1 ($O$)', 'Elbow Joint 2', 'Wrist Joint 3']):
        ax.plot(pt[0], pt[1], 'o', color='#111827', markersize=8, markeredgecolor='white', markeredgewidth=1.5)

    # Reference dashed lines for angles
    # Base X-axis
    ax.plot([0, 0.25], [0, 0], '--', color='#9ca3af', linewidth=1.2)
    # L1 extended
    ext1 = p1 + np.array([0.22 * np.cos(th1), 0.22 * np.sin(th1)])
    ax.plot([p1[0], ext1[0]], [p1[1], ext1[1]], '--', color='#9ca3af', linewidth=1.2)
    # L2 extended
    ext2 = p2 + np.array([0.18 * np.cos(th1 + th2), 0.18 * np.sin(th1 + th2)])
    ax.plot([p2[0], ext2[0]], [p2[1], ext2[1]], '--', color='#9ca3af', linewidth=1.2)

    # Angle arcs
    arc1 = patches.Arc(p0, 0.25, 0.25, angle=0, theta1=0, theta2=np.rad2deg(th1), color='#dc2626', linewidth=1.6)
    ax.add_patch(arc1)
    ax.text(0.14, 0.04, r'$\theta_1$', fontsize=11, fontweight='bold', color='#dc2626')

    arc2 = patches.Arc(p1, 0.22, 0.22, angle=np.rad2deg(th1), theta1=0, theta2=np.rad2deg(th2), color='#dc2626', linewidth=1.6)
    ax.add_patch(arc2)
    ax.text(p1[0] + 0.11, p1[1] + 0.06, r'$\theta_2$', fontsize=11, fontweight='bold', color='#dc2626')

    arc3 = patches.Arc(p2, 0.18, 0.18, angle=np.rad2deg(th1 + th2), theta1=np.rad2deg(th3), theta2=0, color='#dc2626', linewidth=1.6)
    ax.add_patch(arc3)
    ax.text(p2[0] + 0.08, p2[1] - 0.04, r'$\theta_3$', fontsize=11, fontweight='bold', color='#dc2626')

    # End-effector yaw annotation
    ax.annotate(r'$\phi = \theta_1 + \theta_2 + \theta_3 = 0^\circ$', xy=(p3[0], p3[1]), xytext=(p3[0] + 0.05, p3[1] + 0.14),
                arrowprops=dict(arrowstyle="->", color="#1d4ed8", lw=1.5),
                fontsize=9.5, fontweight='bold', color='#1d4ed8',
                bbox=dict(boxstyle="round,pad=0.25", fc="#eff6ff", ec="#bfdbfe", lw=1))

    # Link label text
    ax.text(p0[0] + 0.15, p0[1] + 0.16, r'$L_1 = 0.40\,\text{m}$', fontsize=9.5, fontweight='bold', color='#1e3a8a', rotation=np.rad2deg(th1))
    ax.text(p1[0] + 0.08, p1[1] + 0.15, r'$L_2 = 0.35\,\text{m}$', fontsize=9.5, fontweight='bold', color='#0d9488', rotation=np.rad2deg(th1 + th2) - 10)
    ax.text(p2[0] + 0.04, p2[1] + 0.02, r'$L_3$', fontsize=9.5, fontweight='bold', color='#d97706')

    ax.set_xlim(-0.08, 0.82)
    ax.set_ylim(-0.10, 0.65)
    ax.set_aspect('equal')
    ax.grid(True, linestyle=':', alpha=0.5)
    ax.set_xlabel('Local $X$ [m]', fontsize=9.5)
    ax.set_ylabel('Local $Y$ [m]', fontsize=9.5)
    ax.set_title('(b) Planar Kinematic Coordinate Parameterization', fontsize=10.5, fontweight='bold', pad=6)

def generate_combined_figure(output_path):
    # 1. Get 3D PyBullet render
    img_3d = render_3d_scara()

    # 2. Build 2-panel matplotlib figure
    fig = plt.figure(figsize=(9.2, 4.0), dpi=300)
    
    # Subplot 1: 3D render
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.imshow(img_3d)
    ax1.axis('off')
    ax1.set_title('(a) 3D Articulated SCARA Manipulator Model', fontsize=10.5, fontweight='bold', pad=6)

    # Subplot 2: Kinematic schematic
    ax2 = fig.add_subplot(1, 2, 2)
    render_kinematic_schematic(ax2)

    plt.tight_layout()
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f"Generated SCARA model figure at: {output_path}")

if __name__ == "__main__":
    out = os.path.join("overleaf_package", "figures", "fig2_scara_model.png")
    generate_combined_figure(out)
