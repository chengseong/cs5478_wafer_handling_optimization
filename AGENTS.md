# AGENTS.md: Persistent Operating Contract & Agent Instructions

## 1. Operational Guardrails

> [!IMPORTANT]
> This repository is an academic graduate research project with strict evaluation protocols. The agent operates under the following mandatory constraints:

1. **Interactive Mode (No Autonomous Trial-and-Error):**
   - **Never** execute multi-step autonomous debugging, iterative tuning loops, or speculative test runs without explicit human supervision.
   - Proceed strictly through designated sequential breakpoints.
2. **Execution Protocol (Mandatory Confirmation Gate):**
   - At every breakpoint or code modification stage:
     1. Explain the underlying mathematical and robotics principles from first principles (exam/defense preparation).
     2. Explain the exact minimal code edits proposed and why they address the identified physics failure mode.
     3. **STOP and wait for explicit human confirmation** before touching code or executing simulation scripts.
3. **Code Modification Discipline:**
   - Keep changes focused, surgical, and minimal.
   - Never refactor working infrastructure or modify simulation engine parameters unless mathematically motivated and confirmed by the user.

---

## 2. Project Fundamentals

### System & Payload
- **Robotic Platform:** 3-DOF planar SCARA manipulator (`scara_wafer.urdf`).
  - Link 1 length: $L_1 = 0.40\text{ m}$ (Shoulder, mass $4.5\text{ kg}$)
  - Link 2 length: $L_2 = 0.35\text{ m}$ (Elbow, mass $3.0\text{ kg}$)
  - Link 3 length: $L_3 = 0.15\text{ m}$ (Wrist to wafer center of mass, paddle mass $1.2\text{ kg}$)
  - Revolute joint axes strictly aligned with $+Z$ ($\mathbf{g}(\mathbf{q}) = \mathbf{0}$ for joint torques).
- **Payload:** 300 mm Silicon Wafer ($128\text{ g}$, radius $r = 0.15\text{ m}$).
  - Simulation collision cylinder thickness expanded to $4.0\text{ mm}$ ($2.0\text{ mm}$ half-height) to prevent discrete GJK/EPA contact normal sign inversion (tunneling).
- **Transfer Environment:** FOUP Cassette pick position $[0.45, -0.20, 0.0]$ to Vacuum Chamber target $[0.72, 0.00, 0.0]$ through a narrow slit valve aperture ($350\text{ mm} \times 50\text{ mm}$ at $X = 0.60\text{ m}$).
- **Yaw Invariance:** End-effector yaw strictly constrained to $\phi = \theta_1 + \theta_2 + \theta_3 \equiv 0^\circ$ for aperture clearance.

### Physics & Friction Cone Constraints
- **Contact Mechanism:** Non-prehensile planar manipulation governed solely by Coulomb friction.
- **Critical Friction Acceleration Limit:**
  $$a_{\text{crit}} = \mu_s g = 0.25 \times 9.81\text{ m/s}^2 = 2.4525\text{ m/s}^2$$
- **Engineering Derating Boundary:**
  $$a_{\max} = \eta \mu_s g = 0.85 \times 0.25 \times 9.81\text{ m/s}^2 \approx 2.085\text{ m/s}^2 \quad (\eta = 0.85)$$
- **Performance Threshold:** Maximum link-relative wafer slip offset:
  $$\Delta r_{\text{wafer}} \le 0.100\text{ mm} \quad (100\ \mu\text{m})$$

### Simulation Engine Calibrations (PyBullet)
- **Timestep:** $\Delta t = 1/240\text{ s}$ ($240\text{ Hz}$ fixed frequency).
- **Constraint Solver:** Projected Gauss-Seidel LCP with `numSolverIterations=150`.
- **Friction Model:** True non-linear elliptic cone (`enableConeFriction=1`), restitution $0.0$.
- **Multiplicative Friction Calibration:** $\mu_{\text{paddle}} = 1.0$, $\mu_{\text{wafer}} = 0.25 \implies \mu_{\text{effective}} = 1.0 \times 0.25 = 0.25$.
- **Contact Stabilization:** 120-step static settling phase before motion onset.
- **TOPP-RA Solver Wrapper:** Must specify `solver_wrapper='ecos'` on Windows to prevent Cython buffer dtype mismatch (`INT_t` vs `long`).

---

## 3. Technical State & Progress

| Run | Strategy & Setup | Actuation / Control Mode | Duration | Max Measured Slip | Status | Root Cause / Failure Mode |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Run 0** (Phase 1 Baseline) | 1D Cartesian clamped spline via TOPP-RA | Dynamic box anchor (`JOINT_FIXED`, `erp=0.9`) | 1.060 s | 0.048 mm | **PASS** | Validated PyBullet engine calibrations ($240\text{ Hz}$, $N=150$, $\mu = 0.25$). |
| **Run 1** (Phase 2 Heuristic) | Joint-space spline; decoupled limits $\ddot{q}_i \le a_{\max}/L_i$ | Multi-joint PD position control (`gains=0.8`) | 1.011 s | 49.352 mm | **FAIL** | **Kinematic Coupling Fallacy:** Neglected $\mathbf{J}\ddot{\mathbf{q}}$ vector addition and centripetal term $\dot{\mathbf{J}}\dot{\mathbf{q}}$. Actual tip acceleration reached $\sim 4.5\text{ m/s}^2$ ($180\%$ above Coulomb ceiling). |
| **Run 2** (Phase 2 Derated) | Joint-space spline; $4\times$ derated joint limits | Multi-joint PD position control (`forces=[500, 400, 200]`) | 2.096 s | 19.642 mm | **FAIL** | **Scalar Bound Inadequacy:** Doubled cycle time, but rotational swing and vector superposition still breached the friction cone. Proved scalar joint limits cannot bound 2-norm Cartesian vectors. |
| **Run 3** (Phase 2 Operational) | Cartesian path parameterization under 2-norm acceleration, mapped via IK | Operational feedforward velocity $\dot{\mathbf{q}} = \mathbf{J}^{-1}\dot{\mathbf{x}}$ with decentralized PD (`kp=0.3, kd=1.0`) | 1.114 s | 12.804 mm | **FAIL** | **Dynamic Tracking Ringing:** Trajectory was kinematically bounded ($a_{\text{planned}} \le 2.085\text{ m/s}^2$), but uncompensated manipulator inertia ($\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$) caused phase lag and high-frequency PD restorative torque oscillations ($a_{\text{actual}} \sim 3.4 - 4.1\text{ m/s}^2$). |
| **Run 4** (Pending) | Operational Space TOPP-RA | Computed Torque Control: Inverse Dynamics Feedforward (`p.calculateInverseDynamics` RNEA) | *TBD* | *Target $\le 0.100\text{ mm}$* | *Pending* | Resolves inertial phase lag and servo chatter by canceling nonlinear manipulator dynamics via feedforward. |

### Current Next Step
- **Action:** Breakpoint 1 (Run 4 implementation).
- **Objective:** Update `verify_wafer_transfer_scara.py` to evaluate inverse dynamics $\boldsymbol{\tau}_{\text{ff}} = \mathbf{M}(\mathbf{q}_{\text{des}})\ddot{\mathbf{q}}_{\text{des}} + \mathbf{C}(\mathbf{q}_{\text{des}}, \dot{\mathbf{q}}_{\text{des}})\dot{\mathbf{q}}_{\text{des}}$ using `p.calculateInverseDynamics`, apply feedforward torques to the SCARA motors, stiffen the servos, and verify that measured slip satisfies $\Delta r \le 0.100\text{ mm}$.

---

## 4. Living Document Maintenance Rule

> [!IMPORTANT]
> **Mandatory State Update Protocol:**
> At the conclusion of every simulation experiment or breakpoint, the agent **MUST** automatically:
> 1. Update the telemetry table in **Section 3 of `AGENTS.md`** with the latest empirical quantitative metrics (`Duration`, `Max Measured Slip`, `Status`, `Root Cause / Findings`).
> 2. Synchronize the corresponding benchmark log table in **Section 4.1 of `PRESENTATION_NOTES.md`**.
> 3. Summarize the findings and **halt for explicit user review** before advancing to the next milestone.
