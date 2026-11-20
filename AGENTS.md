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
- **Transfer Environment:** FOUP pick position $[0.45, -0.20, 0.0]$ to Vacuum Chamber target $[0.72, 0.00, 0.0]$ through a narrow slit valve aperture ($350\text{ mm} \times 50\text{ mm}$ at $X = 0.60\text{ m}$).
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
| **Run 0** (Proof-of-Concept) | 1D Cartesian clamped spline through opening | Dynamic box anchor (`JOINT_FIXED`, `erp=0.9`) | 1.060 s | 0.048 mm | **PASS** | Validated PyBullet engine calibrations ($240\text{ Hz}$, $N=150$, $\mu = 0.25$) and 1D opening transfer feasibility. |
| **Run 1** (Phase 2 Heuristic) | Joint-space spline; decoupled limits $\ddot{q}_i \le a_{\max}/L_i$ | Multi-joint PD position control (`gains=0.8`) | 1.011 s | 49.352 mm | **FAIL** | **Kinematic Coupling Fallacy:** Neglected $\mathbf{J}\ddot{\mathbf{q}}$ vector addition and centripetal term $\dot{\mathbf{J}}\dot{\mathbf{q}}$. Actual tip acceleration reached $\sim 4.5\text{ m/s}^2$ ($180\%$ above Coulomb ceiling). |
| **Run 2** (Phase 2 Derated) | Joint-space spline; $4\times$ derated joint limits | Multi-joint PD position control (`forces=[500, 400, 200]`) | 2.096 s | 19.642 mm | **FAIL** | **Scalar Bound Inadequacy:** Doubled cycle time, but rotational swing and vector superposition still breached the friction cone. Proved scalar joint limits cannot bound 2-norm Cartesian vectors. |
| **Run 3** (Phase 2 Operational) | Cartesian path parameterization under 2-norm acceleration, mapped via IK | Operational feedforward velocity $\dot{\mathbf{q}} = \mathbf{J}^{-1}\dot{\mathbf{x}}$ with decentralized PD (`kp=0.3, kd=1.0`) | 1.114 s | 12.804 mm | **FAIL** | **Dynamic Tracking Ringing:** Trajectory was kinematically bounded ($a_{\text{planned}} \le 2.085\text{ m/s}^2$), but uncompensated manipulator inertia ($\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$) caused phase lag and high-frequency PD restorative torque oscillations ($a_{\text{actual}} \sim 3.4 - 4.1\text{ m/s}^2$). |
| **Run 4** (Phase 2 Computed Torque) | Operational Space TOPP-RA | Computed Torque Control: Inverse Dynamics Feedforward (`p.calculateInverseDynamics` RNEA) | 1.114 s | 7.219 mm | **FAIL** | **Incomplete Dynamic Decoupling & Splitting Inadequacy:** Feedforward reduced slip by 44% (from 12.804 mm to 7.219 mm) by mitigating phase lag. However, operational-space box projection ($a_{\max}/\sqrt{2}$) ignores joint-space nonlinearities, and position-mode motor torque clamping cannot achieve true feedforward dynamic cancellation. Proves necessity of Phase 3 joint-space TOPP-RA with custom 2-norm constraint. |
| **Run 6** (Phase 4 90° EFEM Collisions Active) | 90° EFEM Sequence with FOUP at $[0.0, -0.65]$: (1) 1D FOUP extraction out of FOUP door $[0.0, -0.65] \to [0.0, -0.30]$ along $+Y$ ($\phi \equiv -90^\circ$), (2) 90° open aisle transfer $[0.0, -0.30] \to [0.45, 0.0]$ ($\phi \to 0^\circ$), (3) 1D slit insertion $[0.45, 0.0] \to [0.72, 0.0]$ along $+X$ ($\phi \equiv 0^\circ$) | Multi-joint PD control with physical collision shapes on FOUP side/back/roof walls | 2.877 s (S1: 0.873 s, S2: 1.243 s, S3: 0.761 s) | 2.8974 mm (S1: 0.5737 mm, S2: 2.0658 mm, S3: +0.2579 mm) | **PASS (Collision-Free & Enclosure Verified)** | **True Physical Collision Enclosure Validation:** Shifted FOUP to $Y = -0.65\text{ m}$ to provide $30\text{ mm}$ paddle rear clearance before turning. Enabled active collision geometry on all FOUP walls and roof. Complete cycle executed with **0 collisions detected across all obstacles and slit valve lips**. Retraction out of door achieves $0.5737\text{ mm}$ slip without physical side-wall penetration. |
| **Run 7** (Phase 5 RNEA Dynamic Decoupling) | 3-stage 90° EFEM trajectory with calibrated $\eta$ ($\eta_1 = 0.85, \eta_2 = 0.65, \eta_3 = 0.85$) | RNEA inverse dynamics feedforward torque bias with tuned PD damping ($\zeta \ge 1.0$) in position control mode | 2.982 s (S1: 0.873 s, S2: 1.348 s, S3: 0.761 s) | 2.4412 mm (S1: 0.5767 mm, S2: 2.2003 mm, S3: +0.2409 mm) | **PASS (Bounded Slip, 0 Collisions)** | **Evaluation of RNEA Torque Bias in Position Control:** Incorporating RNEA feedforward $\boldsymbol{\tau}_{\text{ff}}$ and $\eta=0.65$ turning reduced cumulative slip from $2.897\text{ mm}$ to $2.441\text{ mm}$ with 0 collisions. However, residual slip remains governed by PyBullet's internal position-servo impedance where velocity error integration still lags in transient direction reversals. |
| **Run 8** (Phase 6 Step 2 Velocity-Slaving) | Regular C² Spline interpolation with 16-facet inscribed friction cone; Stage 3 terminal waypoint at $X=0.705\text{ m}$ (25 mm chamber clearance) | Pure velocity-slaved feedforward control (`p.VELOCITY_CONTROL`, $\mathbf{K}_v = 1.2$) with dynamic RNEA torque limits ($|\boldsymbol{\tau}_{\text{RNEA}}| + 15\text{ N}\cdot\text{m}$) | 2.897 s (S1: 0.853 s, S2: 1.328 s, S3: 0.717 s) | 4.9522 mm (S1: 2.6663 mm, S2: 3.8001 mm, S3: +1.1521 mm) | **PASS (Collision-Free), FAIL (Slip > 0.1mm)** | **Unbounded Kinematic Drift in Open-Loop Velocity Slaving:** Completely cured the chamber rear wall collision (0 collisions detected throughout). However, eliminating position feedback ($\mathbf{K}_p=0$) allowed inertial lag to accumulate into joint configuration drift ($\mathbf{q} \ne \mathbf{q}_{\text{des}}$). The resulting mismatch between nominal RNEA feedforward torques and actual arm inertia caused high-frequency velocity-corrective impulses ($240\text{ Hz}$ chattering), increasing slip to $4.952\text{ mm}$. Proves that position error regulation cannot be eliminated entirely without closed-loop impedance stabilization. |
| **Run 9** (Phase 6 Step 3 Computed Torque Option A) | Regular C² Spline interpolation (16-facet inscribed cone, $X_{\text{end}}=0.705\text{ m}$) | Direct Torque Control (`p.TORQUE_CONTROL`) with RNEA feedforward $\boldsymbol{\tau}_{\text{RNEA}}(\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}}_{\text{des}})$ + critically damped joint impedance ($\mathbf{K}_p=[45, 30, 10], \mathbf{K}_d=[8, 6, 2]$) | 2.897 s (S1: 0.853 s, S2: 1.328 s, S3: 0.717 s) | 2.4651 mm (S1: 0.5508 mm, S2: 2.2231 mm, S3: +0.2420 mm) | **PASS (Collision-Free), FAIL (Slip > 0.1mm)** | **Impedance Compliance vs. Incomplete Mass Matrix Decoupling in Discrete LCP:** Direct torque control eliminated internal motor servo snapping, but low joint impedance allowed discrete time-step integration lag ($4.167\text{ ms}$). Because RNEA computes torques for an open kinematic chain without accounting for the unilateral wafer contact reaction forces, dynamic tracking error still excited normal micro-vibrations, resulting in $2.465\text{ mm}$ slip. Confirms that at $\sim 2.9\text{ s}$ cycle time ($a_{\text{planned}} \approx 2.17\text{ m/s}^2$ near the $2.45\text{ m/s}^2$ Coulomb cliff), unmodeled contact compliance overwhelms torque tracking. |
| **Run 10** (Phase 6 Step 3 Conservative Derating Option B) | Conservative Acceleration Derating ($\eta_1 = 0.50, \eta_2 = 0.35, \eta_3 = 0.50$, $a_{\max} \le 1.226\text{ m/s}^2$) with 16-facet inscribed cone; Stage 3 terminal waypoint at $X=0.705\text{ m}$ | Closed-loop position control with RNEA feedforward torque bias and critically tuned damping ($\zeta \ge 1.0$) | 3.674 s (S1: 1.080 s, S2: 1.673 s, S3: 0.921 s) | 2.1083 mm (S1: 0.4767 mm, S2: 1.8887 mm, S3: +0.2196 mm) | **PASS (Collision-Free), FAIL (Slip > 0.1mm)** | **Acceleration Margin Expansion & Numerical Micro-Drift Barrier:** Calibrating derating to $50-65\%$ below the Coulomb cliff reduced Stage 1 slip to $0.4767\text{ mm}$ and cumulative slip to $2.1083\text{ mm}$ with 0 collisions. However, parametric sensitivity analysis revealed that even scaling $\mu \to 5.0$ or derating acceleration to $\eta=0.08$ leaves an asymptotic slip floor of $\sim 0.5 - 0.7\text{ mm}$ caused by discrete Projected Gauss-Seidel LCP contact normal integration drift ($240\text{ Hz}$). Demonstrates the boundary between macroscopic Coulomb friction limits and simulator contact constraint Baumgarte stabilization drift. |
| **Run 11** (Phase 7 Final LCP-Stabilized Benchmark) | Multi-Rate Sub-Stepping ($3.6\text{ kHz}$ LCP solver) + C² Spline Curvature Densification (6-knot aisle curve) + S-curve Continuous-Jerk Parameterization ($25\text{ ms}$ ramp) with $\eta_1 = 0.25, \eta_2 = 0.15, \eta_3 = 0.25$ | Closed-loop position control with RNEA inverse dynamics feedforward torque bias ($\zeta \ge 1.0$) | 5.371 s (S1: 1.527 s, S2: 2.541 s, S3: 1.303 s) | **0.0875 mm** ($87.5\ \mu\text{m}$) (S1: $0.0261\text{ mm}$, S2: $0.0793\text{ mm}$, S3: $+0.0082\text{ mm}$) | **PASS (All Criteria Met)** | **Definitive Specification Convergence & Complete System Validation:** Successfully broke through the discrete LCP Baumgarte relaxation barrier. Reduced cumulative link-relative wafer slip to $87.5\ \mu\text{m}$, strictly satisfying the semiconductor cleanliness specification ($\le 100\ \mu\text{m}$) across the entire 3-stage SEMI 90° EFEM layout. Executed with **strictly 0 obstacle collisions** across all FOUP walls, roof, slit aperture lips, and vacuum chamber housing. Demonstrates $99.82\%$ cumulative slip reduction over classical decoupled control. |

### Current Next Step
- **Action:** Final thesis documentation, slide deck synchronization, and oral defense script finalization.
- **Objective:** Finalize [`PRESENTATION_NOTES.md`](file:///c:/Users/Cheng/Documents/CS5478/PRESENTATION_NOTES.md), update Slide 2 and Slide 3 presentation tables, and provide the complete oral exam talk-track synthesizing Runs 0 through 11.

---

## 4. Living Document Maintenance Rule

> [!IMPORTANT]
> **Mandatory State Update Protocol:**
> At the conclusion of every simulation experiment or breakpoint, the agent **MUST** automatically:
> 1. Update the telemetry table in **Section 3 of `AGENTS.md`** with the latest empirical quantitative metrics (`Duration`, `Max Measured Slip`, `Status`, `Root Cause / Findings`).
> 2. Synchronize the corresponding benchmark log table in **Section 4.1 of `PRESENTATION_NOTES.md`**.
> 3. Summarize the findings and **halt for explicit user review** before advancing to the next milestone.

