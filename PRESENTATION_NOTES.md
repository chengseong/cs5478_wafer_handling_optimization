# Defense Notes: High-Speed Non-Prehensile Wafer Handling Optimization

## Academic Defense & Technical Reference Manual
**Project:** Autonomous High-Speed Non-Prehensile Transfer of 300 mm Silicon Wafers  
**Manipulator Architecture:** 3-DOF Planar SCARA Manipulator ($L_1 = 0.40\text{ m}, L_2 = 0.35\text{ m}, L_3 = 0.15\text{ m}$)  
**Target Environment:** Semiconductor FOUP Cassette to Vacuum Process Chamber via Slit Valve ($350\text{ mm} \times 50\text{ mm}$)  
**Primary Non-Prehensile Constraint:** Planar Coulomb Friction Cone Threshold ($\mu_s = 0.25$, $\eta = 0.85$, $a_{\max} \approx 2.085\text{ m/s}^2$)  
**Allowable Relative Wafer Displacement Threshold:** $\Delta r_{\text{wafer}} \le 0.100\text{ mm}$ ($100\ \mu\text{m}$)

---

## 1. Mathematical Foundations

### 1.1 Coulomb Friction Cone Mechanics & Derated Safety Boundary
In non-prehensile semiconductor wafer transport, the silicon wafer ($m_w = 0.128\text{ kg}$, $r_w = 0.15\text{ m}$) rests unclamped on an end-effector paddle (vacuum chucking or edge-gripping is avoided to eliminate backside particle contamination and mechanical stress). The coupling between the paddle and the wafer is governed solely by contact normal and tangential Coulomb friction.

#### Contact Equilibrium & d'Alembert Inertial Force
Consider the wafer in an accelerating non-inertial reference frame attached to the end-effector paddle. Let the planar acceleration of the paddle contact interface be $\mathbf{a}_{\text{tip}} = [a_x, a_y]^T \in \mathbb{R}^2$, and vertical acceleration $a_z = 0$ under gravity $g = 9.81\text{ m/s}^2$.
By d'Alembert's principle, the wafer experiences a fictitious inertial body force:
$$\mathbf{F}_{\text{inertial}} = - m_w \mathbf{a}_{\text{tip}}$$

For vertical equilibrium without contact detachment or normal separation:
$$N = m_w g$$

The tangential contact friction force $\mathbf{f}_t \in \mathbb{R}^2$ exerted by the paddle onto the wafer must equilibrate the inertial force to prevent relative motion:
$$\mathbf{f}_t = m_w \mathbf{a}_{\text{tip}}$$

#### Coulomb No-Slip Criterion
Under the classical isotropic Coulomb friction model, slip is avoided if and only if the tangential force lies strictly within the friction cone:
$$\|\mathbf{f}_t\|_2 \le \mu_s N$$
Substituting $\mathbf{f}_t$ and $N$:
$$m_w \|\mathbf{a}_{\text{tip}}\|_2 \le \mu_s m_w g \iff \|\mathbf{a}_{\text{tip}}\|_2 \le \mu_s g$$
For silicon-on-elastomer/polyetheretherketone (PEEK) contact with nominal $\mu_s = 0.25$:
$$a_{\text{coulomb\_limit}} = \mu_s g = 0.25 \times 9.81\text{ m/s}^2 = 2.4525\text{ m/s}^2$$

#### Derated Safety Factor ($\eta = 0.85$) Justification
In physical industrial automation and numerical multi-body physics engines, dynamic disturbances and boundary uncertainties degrade the theoretical limit:
1. **Unmodeled Micro-Vibrations & Structural Flex:** Manipulator joint flexibility, motor cogging torque, and belt harmonic drive compliance generate high-frequency acceleration transients.
2. **Contact Micro-Slip & Stick-Slip Transition:** Pre-sliding displacement occurs before macroscopic sliding; operating directly on the cone boundary $\mu_s g$ triggers irreversible drift through ratcheting.
3. **Normal Force Fluctuations:** Vertical aerodynamic draft and high-speed chamber pressure gradients induce variations in the instantaneous normal force $N(t) = m_w (g + a_z(t))$.

To guarantee zero relative macroscopic displacement ($\le 100\ \mu\text{m}$), an engineering safety factor $\eta = 0.85$ is enforced:
$$a_{\max} = \eta \mu_s g = 0.85 \times 0.25 \times 9.81\text{ m/s}^2 \approx 2.0846\text{ m/s}^2$$
Any planned and executed trajectory must strictly satisfy:
$$\|\mathbf{a}_{\text{tip}}(t)\|_2 = \sqrt{\ddot{x}(t)^2 + \ddot{y}(t)^2} \le a_{\max} \approx 2.085\text{ m/s}^2, \quad \forall t \in [0, T]$$

---

### 1.2 Kinematics of the 3-DOF Planar SCARA Manipulator

The planar robot comprises three revolute joints with axes parallel to $\hat{\mathbf{z}}$:
- Link 1 length $l_1 = 0.40\text{ m}$ (shoulder joint $\theta_1$)
- Link 2 length $l_2 = 0.35\text{ m}$ (elbow joint $\theta_2$)
- Link 3 length $l_3 = 0.15\text{ m}$ (wrist joint $\theta_3$, from wrist pivot to wafer center of mass)

The joint configuration vector is $\mathbf{q} = [\theta_1, \theta_2, \theta_3]^T \in \mathbb{R}^3$.

#### Forward Kinematics (FK)
Let $\theta_{12} = \theta_1 + \theta_2$ and $\theta_{123} = \theta_1 + \theta_2 + \theta_3$. The operational-space pose of the wafer center $\mathbf{x} = [x, y, \phi]^T$ is:
$$\begin{aligned}
x(\mathbf{q}) &= l_1 \cos\theta_1 + l_2 \cos\theta_{12} + l_3 \cos\theta_{123} \\
y(\mathbf{q}) &= l_1 \sin\theta_1 + l_2 \sin\theta_{12} + l_3 \sin\theta_{123} \\
\phi(\mathbf{q}) &= \theta_1 + \theta_2 + \theta_3 = \theta_{123}
\end{aligned}$$
where $\phi$ is the global orientation (yaw angle) of the end-effector paddle in the world frame.

#### Wrist Yaw Decoupling ($\phi \equiv 0^\circ$)
Transfer through the slit valve aperture ($350\text{ mm}$ width) requires the rectangular paddle ($320\text{ mm}$ width) and wafer ($300\text{ mm}$ diameter) to maintain a fixed zero-yaw angle ($\phi \equiv 0^\circ$) relative to the chamber normal:
$$\phi = \theta_1 + \theta_2 + \theta_3 = 0 \iff \theta_3 = -(\theta_1 + \theta_2)$$
When $\phi = 0$, $\cos\theta_{123} = 1$ and $\sin\theta_{123} = 0$. The wafer position reduces to:
$$\begin{aligned}
x &= l_1 \cos\theta_1 + l_2 \cos\theta_{12} + l_3 \\
y &= l_1 \sin\theta_1 + l_2 \sin\theta_{12}
\end{aligned}$$

#### Analytical Inverse Kinematics (IK)
Given a desired Cartesian pose $\mathbf{x}_{\text{des}} = [x, y, \phi]^T$:
1. **Compute Wrist Pivot Coordinates $(x_w, y_w)$:**
   $$x_w = x - l_3 \cos\phi, \quad y_w = y - l_3 \sin\phi$$
2. **Radial Distance Squared:**
   $$r_w^2 = x_w^2 + y_w^2$$
3. **Elbow Angle $\theta_2$ (Law of Cosines):**
   $$D = \frac{r_w^2 - l_1^2 - l_2^2}{2 l_1 l_2}$$
   Workspace feasibility requires $|D| \le 1$. Selecting the elbow-up configuration ($\sigma = +1$):
   $$\theta_2 = \text{atan2}\left(\sqrt{1 - D^2}, D\right)$$
4. **Shoulder Angle $\theta_1$:**
   $$\theta_1 = \text{atan2}(y_w, x_w) - \text{atan2}(l_2 \sin\theta_2, l_1 + l_2 \cos\theta_2)$$
5. **Wrist Angle $\theta_3$:**
   $$\theta_3 = \phi - (\theta_1 + \theta_2)$$

---

### 1.3 Differential & Acceleration Kinematics

#### Operational-Space Jacobian
Differentiating forward kinematics with respect to time yields the velocity mapping $\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}$:
$$\mathbf{J}(\mathbf{q}) = \begin{bmatrix}
-l_1 s_1 - l_2 s_{12} - l_3 s_{123} & -l_2 s_{12} - l_3 s_{123} & -l_3 s_{123} \\
 l_1 c_1 + l_2 c_{12} + l_3 c_{123} &  l_2 c_{12} + l_3 c_{123} &  l_3 c_{123} \\
 1 & 1 & 1
\end{bmatrix}$$
where $s_i = \sin\theta_i$, $c_i = \cos\theta_i$, $s_{ij} = \sin(\theta_i + \theta_j)$, $c_{ij} = \cos(\theta_i + \theta_j)$, $s_{123} = \sin\phi$, $c_{123} = \cos\phi$.

#### Acceleration Kinematics & The Centripetal/Coriolis Term
Differentiating $\dot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\dot{\mathbf{q}}$ with respect to time:
$$\mathbf{a}_{\text{tip}} = \ddot{\mathbf{x}} = \mathbf{J}(\mathbf{q}) \ddot{\mathbf{q}} + \dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}}) \dot{\mathbf{q}}$$

The analytical Jacobian time derivative $\dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}})$ is obtained by chain rule differentiation with $\omega_1 = \dot{\theta}_1$, $\omega_{12} = \dot{\theta}_1 + \dot{\theta}_2$, $\omega_{123} = \dot{\theta}_1 + \dot{\theta}_2 + \dot{\theta}_3$:
$$\dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}}) = \begin{bmatrix}
-l_1 c_1 \omega_1 - l_2 c_{12} \omega_{12} - l_3 c_{123} \omega_{123} & -l_2 c_{12} \omega_{12} - l_3 c_{123} \omega_{123} & -l_3 c_{123} \omega_{123} \\
-l_1 s_1 \omega_1 - l_2 s_{12} \omega_{12} - l_3 s_{123} \omega_{123} & -l_2 s_{12} \omega_{12} - l_3 s_{123} \omega_{123} & -l_3 s_{123} \omega_{123} \\
 0 & 0 & 0
\end{bmatrix}$$

#### Physical Meaning of $\dot{\mathbf{J}}\dot{\mathbf{q}}$
The term $\dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$ represents the convective acceleration of the kinematic chain in operational space. It encapsulates:
1. **Centripetal accelerations:** Proportional to squared joint velocities (e.g., $-l_i \omega_i^2 \hat{\mathbf{r}}$), directed radially inward along the arm segments.
2. **Coriolis cross-coupling:** Proportional to cross-products of velocities from different links ($2 \omega_i \omega_j$).

Even if joint accelerations are zero ($\ddot{\mathbf{q}} = \mathbf{0}$), high joint velocities produce non-zero Cartesian tip acceleration:
$$\mathbf{a}_{\text{tip}}\Big|_{\ddot{\mathbf{q}} = \mathbf{0}} = \dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} \ne \mathbf{0}$$
Failing to account for $\dot{\mathbf{J}}\dot{\mathbf{q}}$ when checking friction limits is a principal root cause of non-prehensile handling failure.

---

### 1.4 Time-Optimal Path Parameterization (TOPP-RA) Mechanics

Given a geometric path $\mathbf{s} \mapsto \mathbf{p}(s) \in \mathbb{R}^m$ for $s \in [0, s_{\text{end}}]$, time-optimal trajectory planning seeks a time reparameterization $s(t)$ that minimizes total travel duration $T$ subject to kinematic and dynamic constraints.

#### Phase-Plane Transformation
Following Pham & Pham (2018), define the phase-plane variables:
$$w(s) = \dot{s}^2, \quad u(s) = \ddot{s} = \frac{1}{2} \frac{dw}{ds}$$
Path velocity and acceleration vectors transform linearly in $(w, u)$:
$$\begin{aligned}
\dot{\mathbf{p}}(s) &= \mathbf{p}'(s) \dot{s} \\
\ddot{\mathbf{p}}(s) &= \mathbf{p}''(s) \dot{s}^2 + \mathbf{p}'(s) \ddot{s} = \mathbf{p}''(s) w(s) + \mathbf{p}'(s) u(s)
\end{aligned}$$
where $\mathbf{p}'(s) = \frac{d\mathbf{p}}{ds}$ and $\mathbf{p}''(s) = \frac{d^2\mathbf{p}}{ds^2}$ are geometric path derivatives.

#### Convex Reachability Analysis
By discretizing the path grid $0 = s_0 < s_1 < \dots < s_N = s_{\text{end}}$, the infinite-dimensional time-optimal control problem is transformed into a finite sequence of small Linear Programs (LPs):
1. **Backward Pass (Controllable Sets):** Computes the maximum admissible terminal velocity profile $w_i \le w_{i,\max}$ such that future constraints remain feasible.
2. **Forward Pass (Reachable Sets & Integration):** Maximizes $w_i$ forward along the path:
   $$\max_{u_i, w_{i+1}} w_{i+1} \quad \text{s.t.} \quad w_{i+1} = w_i + 2 u_i \Delta s_i, \quad (u_i, w_i) \in \mathcal{K}_i$$
   where $\mathcal{K}_i$ is the convex constraint polytope at grid point $s_i$.
Total duration is obtained by numeric quadrature:
$$T = \int_0^{s_{\text{end}}} \frac{1}{\sqrt{w(s)}} \, ds = \sum_{i=0}^{N-1} \frac{2 \Delta s_i}{\sqrt{w_i} + \sqrt{w_{i+1}}}$$

---

## 2. Simulation Calibration Rationale (PyBullet Engine Mechanics)

Accurately simulating non-prehensile contact in a discrete-time rigid body engine requires careful parameter calibration to prevent solver artifacts.

### 2.1 Wafer Geometric Thickness Expansion ($4.0\text{ mm}$ vs $0.775\text{ mm}$)
A physical 300 mm silicon wafer (SEMI M1 standard) has a thickness of $775\ \mu\text{m}$ ($0.775\text{ mm}$). However, in rigid-body physics engines utilizing discrete contact solvers (Bullet, PhysX, ODE):
1. **Contact Normal Inversion (Tunneling):** At discrete timesteps $\Delta t = 1/240\text{ s}$, small vertical displacements or penalty solver penetrations ($10 - 50\ \mu\text{m}$) can cause collision detection algorithms (GJK / EPA) to penetrate past the object's midplane. This causes the computed contact normal to flip sign ($\hat{\mathbf{n}} \to -\hat{\mathbf{n}}$), resulting in the wafer being ejected or snagging violently on the paddle boundary.
2. **Contact Patch Stability:** Setting the collision cylinder height to $4.0\text{ mm}$ ($2.0\text{ mm}$ half-extent) provides adequate penetration buffer depth while preserving the exact physical mass ($m_w = 0.128\text{ kg}$) and planar contact manifold area.

### 2.2 Multiplicative Contact Friction in PyBullet
PyBullet does not use single-surface friction properties; rather, it computes the effective friction coefficient between two contacting bodies via multiplication:
$$\mu_{\text{effective}} = \mu_{\text{body1}} \times \mu_{\text{body2}}$$
To establish an exact physical Coulomb friction coefficient of $\mu_{\text{effective}} = 0.25$:
- Paddle link lateral friction: $\mu_{\text{paddle}} = 1.0$
- Wafer body lateral friction: $\mu_{\text{wafer}} = 0.25$
$$\mu_{\text{effective}} = 1.0 \times 0.25 = 0.250$$
Additionally, `enableConeFriction=1` is activated in `p.setPhysicsEngineParameter` to enforce true elliptic cone friction rather than a decoupled pyramid approximation. Restitution is set to $0.0$ to prevent bounce.

### 2.3 Timestep Selection ($240\text{ Hz}$) & Contact Cache Mechanics
1. **Timestep $\Delta t = 1/240\text{ s} \approx 4.167\text{ ms}$:** Balances numerical stability and computational throughput. At maximum transfer velocity $v_{\max} \approx 1.0\text{ m/s}$, the wafer traverses $\sim 4.17\text{ mm}$ per timestep.
2. **Contact Cache Invalidation & Solver Iterations:** Bullet caches contact points across frames using warm starting. Under high rotational rates or rapid joint reversals, stale contact manifolds introduce phantom friction impulses. To eliminate this:
   - `numSolverIterations=150` is specified to ensure convergence of the Projected Gauss-Seidel (PGS) LCP contact solver.
   - A pre-motion stabilization phase (120 idle steps) is executed before motion onset, allowing the wafer's vertical contact manifold to settle under gravity to its exact static equilibrium normal force ($N = m_w g = 1.2557\text{ N}$).

---

## 3. The Experimental Failure Log & Telemetry Analysis

Every experimental run demonstrates a distinct failure mode and serves as a core milestone for defense:

| Run | Setup & Planning Strategy | Control / Actuation Method | Trajectory Duration | Max Measured Slip | Result | Root Cause / Failure Mechanism |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Run 0** (Phase 1 Baseline) | 1D Cartesian clamped spline via TOPP-RA | Dynamic box anchor (`JOINT_FIXED`, `erp=0.9`) | $1.060\text{ s}$ | $0.048\text{ mm}$ | **PASS** | Validated PyBullet engine calibrations ($240\text{ Hz}$, `numSolverIterations=150`, non-linear cone friction `enableConeFriction=1`, $\mu_{\text{paddle}}=1.0, \mu_{\text{wafer}}=0.25$). |
| **Run 1** (Phase 2 Heuristic) | Joint-space spline. Decoupled joint acceleration limits ($\ddot{q}_i \le a_{\max}/L_i$) | Multi-joint PD position control (`gains=0.8`) | $1.011\text{ s}$ | $49.352\text{ mm}$ | **FAIL** | **Kinematic Coupling Fallacy:** Neglected vector addition $\mathbf{J}\ddot{\mathbf{q}}$ and centripetal term $\dot{\mathbf{J}}\dot{\mathbf{q}}$. Commanded tip acceleration reached $\sim 4.5\text{ m/s}^2$ ($180\%$ above Coulomb ceiling). Matches d'Alembert prediction ($49.6\text{ mm}$). |
| **Run 2** (Phase 2 Derated) | Joint-space spline. Heuristic $4\times$ derated joint limits | Multi-joint PD position control (`forces=[500, 400, 200]`) | $2.096\text{ s}$ | $19.642\text{ mm}$ | **FAIL** | **Scalar Bound Inadequacy:** Doubled cycle time, but rotational swing and vector superposition still breached the friction cone. Proved scalar joint limits cannot bound 2-norm Cartesian vectors. |
| **Run 3** (Phase 2 Operational) | Cartesian path parameterization under 2-norm acceleration, mapped via analytical IK | Operational feedforward velocity $\dot{\mathbf{q}} = \mathbf{J}^{-1}\dot{\mathbf{x}}$ with decentralized PD (`kp=0.3, kd=1.0`) | $1.114\text{ s}$ | $12.804\text{ mm}$ | **FAIL** | **Dynamic Tracking Ringing:** Trajectory was kinematically bounded ($a_{\text{planned}} \le 2.085\text{ m/s}^2$), but uncompensated manipulator inertia ($\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$) caused phase lag and high-frequency PD restorative torque oscillations ($a_{\text{actual}} \sim 3.4 - 4.1\text{ m/s}^2$). |
| **Run 4** (Pending) | Operational Space TOPP-RA | Computed Torque Control: Inverse Dynamics Feedforward (`p.calculateInverseDynamics` RNEA) | *TBD* | *Target $\le 0.10\text{ mm}$* | *Pending* | Resolves inertial phase lag and servo chatter by canceling nonlinear manipulator dynamics. |

---

### Detailed Theoretical Explanations of Failure Mechanisms

#### Run 1: The Kinematic Coupling Fallacy
- **Approach:** Decoupled joint limits $\ddot{\theta}_1 \le a_{\max}/(l_1+l_2+l_3)$, $\ddot{\theta}_2 \le a_{\max}/(l_2+l_3)$, $\ddot{\theta}_3 \le a_{\max}/l_3$.
- **Why It Failed:** Scalar joint bounds assume linear superposition along independent axes. In reality, Cartesian acceleration is the vector sum:
  $$\mathbf{a}_{\text{tip}} = \mathbf{J}\ddot{\mathbf{q}} + \dot{\mathbf{J}}\dot{\mathbf{q}}$$
  When joints 1 and 2 rotate simultaneously, their tangential acceleration vectors constructively align. Furthermore, link 1 rotation creates centripetal acceleration on link 2 and the paddle:
  $$a_{\text{centripetal}} \sim \omega_1^2 (l_1 + l_2 \cos\theta_2)$$
  The vector magnitude peaked at $4.5\text{ m/s}^2$, exceeding the friction limit by $180\%$, resulting in gross wafer slippage of $49.352\text{ mm}$.

#### Run 2: The Inadequacy of Scalar Derating
- **Approach:** Artificially slashed joint acceleration bounds by $4\times$ to compensate for coupling.
- **Why It Failed:** While cycle time doubled from $1.011\text{ s}$ to $2.096\text{ s}$, the trajectory path shape was not constrained. During the maximum curvature arc of the transfer, the combination of centripetal acceleration $\dot{\mathbf{J}}\dot{\mathbf{q}}$ and coordinate projection still breached the $2.085\text{ m/s}^2$ envelope, resulting in $19.642\text{ mm}$ of slip. This proved that **no set of fixed scalar joint limits can optimally bound a configuration-dependent 2-norm Cartesian vector**.

#### Run 3: Dynamic Tracking Ringing (Uncompensated Manipulator Dynamics)
- **Approach:** Parameterized the trajectory directly in Cartesian space with component constraints guaranteeing $\|\mathbf{a}_{\text{planned}}\|_2 \le 2.085\text{ m/s}^2$. Used kinematic inversion for feedforward joint velocities $\dot{\mathbf{q}}_{\text{ff}} = \mathbf{J}^{-1}\dot{\mathbf{x}}_{\text{des}}$ and low-gain decentralized PD position servos (`kp=0.3, kd=1.0`).
- **Why It Failed:** The robot links possess significant mass ($m_1 = 4.5\text{ kg}, m_2 = 3.0\text{ kg}, m_3 = 1.2\text{ kg}$). The full rigid-body equations of motion are:
  $$\boldsymbol{\tau} = \mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}$$
  Because no inertial feedforward torque was commanded, the low-gain servos lagged significantly behind the desired acceleration profile. As tracking error accumulated, the proportional-derivative control laws applied rapid restorative torques, causing high-frequency mechanical oscillation ("servo chatter / ringing"). Actual tip acceleration spiked to $3.4 - 4.1\text{ m/s}^2$, violently breaching the Coulomb cone and causing $12.804\text{ mm}$ of slip.

---

## 4. Oral Defense FAQ (Anticipated Examination Questions)

**Q1: Why is non-prehensile manipulation preferred over vacuum or mechanical clamping in semiconductor wafer transfer?**  
*Answer:* Mechanical clamping induces localized edge chipping, lattice stress fractures, and micro-particle generation. Vacuum chucking creates backside surface contamination and can cause localized wafer deformation that ruins photolithography focal depth. Non-prehensile planar transport eliminates these mechanisms entirely, provided inertial forces remain strictly within the Coulomb friction cone.

**Q2: What is the significance of the $\dot{\mathbf{J}}\dot{\mathbf{q}}$ term in high-speed robot control?**  
*Answer:* $\dot{\mathbf{J}}\dot{\mathbf{q}}$ accounts for centripetal and Coriolis accelerations induced by joint velocities. In high-speed handling, ignoring $\dot{\mathbf{J}}\dot{\mathbf{q}}$ leads to severe underestimation of tip acceleration, even when joint accelerations $\ddot{\mathbf{q}}$ are small or zero.

**Q3: How does Computed Torque Control prevent servo ringing in non-prehensile handling?**  
*Answer:* By computing the inverse dynamics torques $\boldsymbol{\tau}_{\text{ff}} = \mathbf{M}(\mathbf{q}_{\text{des}})\ddot{\mathbf{q}}_{\text{des}} + \mathbf{C}(\mathbf{q}_{\text{des}}, \dot{\mathbf{q}}_{\text{des}})\dot{\mathbf{q}}_{\text{des}}$ via the Recursive Newton-Euler Algorithm (RNEA), the controller feeds forward the exact torques required to accelerate the manipulator links. This decouples and linearizes the multi-body dynamics, preventing the phase lag and high-gain restorative torque spikes that excite contact vibration and break Coulomb stiction.

