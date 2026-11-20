# Wafer Transfer Optimization

[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)
[![PyBullet](https://img.shields.io/badge/PyBullet-3.25-green.svg)](https://pybullet.org/)
[![TOPP-RA](https://img.shields.io/badge/TOPP--RA-0.6.0-orange.svg)](https://github.com/hungpham2511/toppra)
[![Target Slip](https://img.shields.io/badge/Wafer%20Slip-87.5%20%CE%BCm%20%28PASS%29-brightgreen.svg)]()

> **CS5478 Final Project — National University of Singapore (NUS)**  
> **Author:** Ong Cheng Seong (`e0406228@u.nus.edu`)  
> **Repository:** [https://github.com/chengseong/cs5478_wafer_handling_optimization](https://github.com/chengseong/cs5478_wafer_handling_optimization)  
> **Academic Report:** 5–8 Page NeurIPS Conference Paper format ([Overleaf Package](overleaf_package/) / [Package ZIP](CS5478_Wafer_Transfer_Overleaf_Package.zip))

---

## 1. Project Overview

In advanced semiconductor manufacturing (FEOL), Equipment Front End Modules (EFEM) transfer 300 mm silicon wafers across atmospheric cleanrooms into vacuum process load-locks. Mechanical clamping mechanisms and high-vacuum suction cups are strictly prohibited on polished active wafer layers due to micro-scratching and particulate shedding. Transport must occur via **non-prehensile planar manipulation**, where the wafer rests freely on a flat ceramic end-effector paddle retained exclusively by dry Coulomb friction ($\mu_s = 0.25$, $a_{\text{crit}} = \mu_s g = 2.4525\text{ m/s}^2$).

The industry cleanliness specification demands:
$$\Delta r_{\text{wafer}} \le 100\ \mu\text{m} \quad (0.100\text{ mm})$$
with **strictly zero collisions** across narrow cassette enclosures and a $350\text{ mm} \times 50\text{ mm}$ vacuum slit aperture.

### Key Contributions & Results:
1. **Kinematic Coupling Fallacy Proof:** Proved analytically and empirically that classical decoupled joint limits ($\ddot{q}_i \le a_{\max}/L_i$) fail because revolute vector summation $\mathbf{J}\ddot{\mathbf{q}}$ and quadratic centripetal acceleration $\dot{\mathbf{J}}\dot{\mathbf{q}}$ breach the friction cone by $180\%$, inducing $49.352\text{ mm}$ slip (Run 1).
2. **Joint-Space TOPP-RA with 16-Facet Inscribed Polyhedral Linearization:** Formulated a canonical second-order cone constraint parameterized directly along joint splines $\mathbf{q}(s)$ linearized via an $M=16$ inscribed polygon, combined with Recursive Newton-Euler (RNEA) dynamic feedforward torques.
3. **Discrete LCP Baumgarte Creep Discovery:** Discovered and mathematically proved that residual millimetre-scale slip in rigid-body physics engines is an $\mathcal{O}(\Delta t)$ numerical integration artifact of Projected Gauss-Seidel (PGS) Linear Complementarity Problem (LCP) Baumgarte error reduction ($ERP$).
4. **Final System Validation (Run 11):** By synthesizing $C^2$ spline curvature densification, continuous-jerk S-curve boundary conditioning ($25\text{ ms}$), and multi-rate LCP sub-stepping ($3.6\text{ kHz}$), achieved an empirical wafer slip of **$87.5\ \mu\text{m}$** ($< 100\ \mu\text{m}$ cleanroom threshold) with **strictly 0 obstacle collisions** across the entire SEMI 90° EFEM layout ($99.82\%$ cumulative slip reduction).

---

## 2. Repository Architecture

```
.
├── README.md                               # Project documentation & run guide for teaching team
├── AGENTS.md                               # Persistent technical log & 12-run empirical telemetry table
├── PRESENTATION_NOTES.md                   # First-principles defense & oral exam manual
├── CS5478_Wafer_Transfer_Overleaf_Package.zip # Self-contained Overleaf upload zip archive
│
├── scara_wafer.urdf                        # Calibrated 3-DOF SCARA model & thin blade geometry
├── scara_kinematics.py                     # Analytic FK, IK, Jacobian, and Jacobian derivative
├── cartesian_friction_constraint.py        # Custom TOPP-RA 16-facet polyhedral friction cone constraint
│
├── verify_phase1_baseline.py               # Run 0: 1D clamped spline proof-of-concept
├── verify_phase2_baselines.py              # Runs 1-4: Decoupled limits, scalar derating, PD lag, computed torque
├── verify_phase3_joint_toppra.py           # Run 5: Joint-space TOPP-RA formulation
├── verify_phase4_multisegment.py           # Run 6: 3-stage SEMI 90° EFEM with active collision enclosure
├── verify_phase5_decoupled_efem.py         # Run 7: RNEA torque bias in position control
├── verify_phase6_step1_quintic_smoothing.py# Run 8 prep: Boundary acceleration continuity
├── verify_phase6_step2_velocity_slaving.py # Run 8: Pure velocity slaving (kinematic drift analysis)
├── verify_phase6_step3_torque_control.py   # Run 9: Direct joint torque impedance control
├── verify_phase6_step3_option_b_derating.py# Run 10: Conservative derating & LCP creep floor discovery
├── verify_phase7_lcp_stabilized_transfer.py# Run 11: Final Multi-Rate (3.6 kHz) LCP benchmark (PASS: 87.5 µm)
│
├── interactive_simulation_viewer.py        # Interactive GUI simulation viewer with step slider & camera views
├── render_environment_setup.py             # High-resolution 2D kinematic corridor & 3D overview renderer
├── run_milestone_demo.py                   # Automated test harness running baselines vs. final benchmark
│
└── overleaf_package/                       # NeurIPS LaTeX Paper submission package
    ├── main.tex                            # LaTeX paper source (5–8 pages, NeurIPS format)
    ├── neurips_2024.sty                    # NeurIPS conference style
    ├── references.bib                      # Validated IEEEtran bibliography database (14 papers)
    └── figures/                            # High-resolution publication figures
        ├── fig1_efem_kinematic_map.png
        ├── fig2_scara_model.png
        ├── fig2_behind_scara_progression.png
        └── fig3_phase7_acceleration_slip_profile.png
```

---

## 3. Environment Setup & Installation

### Prerequisites
- Python 3.10 or 3.11 (tested on Windows 11 and Linux)
- Conda or virtual environment manager

### Step-by-Step Installation

```bash
# 1. Clone the repository
git clone https://github.com/chengseong/cs5478_wafer_handling_optimization.git
cd cs5478_wafer_handling_optimization

# 2. Create and activate a dedicated conda environment
conda create -n cs5478 python=3.10 -y
conda activate cs5478

# 3. Install core robotics & simulation dependencies
pip install pybullet numpy scipy matplotlib pillow
pip install toppra ecos
```

> [!NOTE]
> On Windows, `toppra` must execute with `solver_wrapper='ecos'` to avoid Cython memory buffer type mismatches (`INT_t` vs `long`), which is already pre-configured across all verification scripts.

---

## 4. How to Reproduce Experimental Results

### Quick Start: Run the Final Passing Benchmark (Run 11)
To execute the multi-rate ($3.6\text{ kHz}$) LCP-stabilized cleanroom wafer transfer with live 3D visualization and real-time link-relative slip logging:

```bash
python verify_phase7_lcp_stabilized_transfer.py
```
**Expected Console Output:**
```
================================================================================
PHASE 7 FINAL LCP-STABILIZED TRANSFER BENCHMARK RESULTS:
  Total Execution Duration:  5.371 s
  Stage 1 Slip (FOUP Door):  0.0261 mm (26.1 um)
  Stage 2 Slip (Aisle Turn): 0.0793 mm (79.3 um)
  Final Cumulative Slip:     0.0875 mm (87.5 um)
  Target Specification:      <= 0.100 mm (100 um)
  Total Obstacle Collisions: 0
  VERDICT:                   PASS (All Industrial Cleanroom Criteria Satisfied)
================================================================================
```

---

### Reproduce Key Milestones & Ablation Runs

| Script | Run Evaluated | Strategy & Physics Mode | Expected Slip | Status |
| :--- | :--- | :--- | :--- | :--- |
| `verify_phase1_baseline.py` | Run 0 | 1D clamped spline through opening (box anchor) | $0.048\text{ mm}$ | **PASS** |
| `verify_phase2_baselines.py` | Runs 1–4 | Decoupled bounds, $4\times$ derating, PD lag, computed torque | $49.35\text{ mm} \to 7.22\text{ mm}$ | **FAIL** |
| `verify_phase3_joint_toppra.py` | Run 5 | Joint-space TOPP-RA with 8-facet cone | $0.912\text{ mm}$ | **Analysis** |
| `verify_phase4_multisegment.py` | Run 6 | 3-stage SEMI 90° EFEM layout (active collision geometry) | $2.897\text{ mm}$ | **0 Collisions** |
| `verify_phase5_decoupled_efem.py` | Run 7 | RNEA feedforward torque bias ($\eta_2 = 0.65$) | $2.441\text{ mm}$ | **0 Collisions** |
| `verify_phase6_step2_velocity_slaving.py`| Run 8 | Pure velocity-mode control ($K_p = 0$) | $4.952\text{ mm}$ | **FAIL (Drift)** |
| `verify_phase6_step3_torque_control.py` | Run 9 | Direct joint torque control + impedance damping | $2.465\text{ mm}$ | **FAIL** |
| `verify_phase6_step3_option_b_derating.py`| Run 10 | Conservative acceleration derating ($\eta = 0.50/0.35$) | $2.108\text{ mm}$ | **FAIL (Creep Floor)** |
| `verify_phase7_lcp_stabilized_transfer.py`| Run 11 | Multi-rate ($3.6\text{ kHz}$) + S-curve continuous jerk | **$0.0875\text{ mm}$** ($87.5\ \mu\text{m}$) | **PASS** |

---

### Interactive Visual Simulation & Telemetry Tools

1. **Interactive Simulation Viewer (GUI with Camera & Playback Controls):**
   ```bash
   python interactive_simulation_viewer.py
   ```
2. **Re-generate 2D Kinematic Corridor Map & 3D Environment Overviews:**
   ```bash
   python render_environment_setup.py
   ```
3. **Inspect or Compile NeurIPS Conference Paper:**
   The full academic manuscript is provided in [`overleaf_package/`](overleaf_package/) and packaged as [`CS5478_Wafer_Transfer_Overleaf_Package.zip`](CS5478_Wafer_Transfer_Overleaf_Package.zip) for one-click upload to Overleaf. It compiles cleanly with zero undefined references using pdfLaTeX and BibTeX.

---

## 5. Summary Benchmark Telemetry

| Run | Strategy & Setup | Actuation / Control Mode | Duration | Max Measured Slip | Status | Root Cause / Key Finding |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Run 0** | 1D Cartesian Clamped Spline | Dynamic Box Anchor | 1.060 s | 0.048 mm | **PASS** | Validated PyBullet physics calibrations ($240\text{ Hz}$, $N=150$, $\mu = 0.25$). |
| **Run 1** | Decoupled Limits ($\ddot{q}_i \le a_{\max}/L_i$) | Multi-joint PD Position | 1.011 s | 49.352 mm | **FAIL** | **Kinematic Coupling Fallacy:** Vector superposition $\mathbf{J}\ddot{\mathbf{q}}$ and centripetal term $\dot{\mathbf{J}}\dot{\mathbf{q}}$ breached friction cone by $180\%$. |
| **Run 2** | Uniform $4\times$ Scalar Derating | Multi-joint PD Position | 2.096 s | 19.642 mm | **FAIL** | **Scalar Bound Inadequacy:** Doubled cycle time without bounding Euclidean 2-norm Cartesian vectors. |
| **Run 3** | Cartesian 2-Norm Spline via IK | Operational Velocity + PD | 1.114 s | 12.804 mm | **FAIL** | **Dynamic Tracking Ringing:** Uncompensated inertia caused phase lag and high-frequency PD restorative ringing. |
| **Run 4** | Operational TOPP-RA + Feedforward | RNEA Computed Torque Bias | 1.114 s | 7.219 mm | **FAIL** | **Incomplete Decoupling:** Feedforward reduced slip by $44\%$, but operational box projection ignores joint nonlinearities. |
| **Run 5** | Joint TOPP-RA (8-Facet Cone) | RNEA Bias + Filtering | 0.981 s | 0.912 mm | **Analysis** | Direct joint parameterization eliminates kinematic coupling in planar swing. |
| **Run 6** | 3-Stage SEMI 90° Cleanroom Layout | Active Enclosure Collision Walls | 2.877 s | 2.897 mm | **0 Collisions** | Verified physical collision clearance across FOUP cassette and $350\text{ mm} \times 50\text{ mm}$ vacuum slit. |
| **Run 7** | Inertia Decoupling ($\eta_2 = 0.65$) | RNEA Bias + Tuned Damping | 2.982 s | 2.441 mm | **0 Collisions** | Reduced slip with 0 collisions, but residual slip bounded by motor servo impedance lag. |
| **Run 8** | Pure Velocity Slaving ($K_p = 0$) | Velocity Mode + Torque Limits | 2.897 s | 4.952 mm | **FAIL** | **Kinematic Drift:** Inertial lag accumulated into joint configuration error ($\mathbf{q} \ne \mathbf{q}_{\text{des}}$), causing velocity chattering. |
| **Run 9** | Direct Joint Torque Control | RNEA + Joint Impedance | 2.897 s | 2.465 mm | **FAIL** | **Contact Compliance:** Open-chain RNEA omits unilateral contact forces, exciting discrete normal micro-vibrations. |
| **Run 10** | Conservative Derating ($\eta = 0.50/0.35$) | Closed-Loop RNEA Position | 3.674 s | 2.108 mm | **FAIL** | **LCP Solver Drift Floor:** Asymptotic slip floor caused by discrete Projected Gauss-Seidel LCP contact integration. |
| **Run 11** | **Multi-Rate ($3.6\text{ kHz}$) + S-Curve Ramp** | **Closed-Loop RNEA Position** | **5.371 s** | **0.0875 mm** ($87.5\ \mu\text{m}$) | **PASS** | **Definitive Specification Convergence:** Satisfies cleanroom standard ($\le 100\ \mu\text{m}$) with strictly 0 obstacle collisions. $99.82\%$ cumulative slip reduction. |

---

## 6. Declaration on the Use of Generative AI

In compliance with the **National University of Singapore (NUS) Policy on Academic Integrity and the Use of Generative AI Tools in Coursework**:

> The author acknowledges the use of Generative AI tools (specifically Google Gemini / Antigravity) during the technical preparation of this project and report.
