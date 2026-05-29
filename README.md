# OpenSeesPy-Seismic-Behavior-Evaluation-Dashboard-for-RC-Columns
This repository contains an open-source structural engineering pipeline developed in Python using OpenSeesPy and Matplotlib. The framework performs a advanced nonlinear cyclic displacement-control analysis of a cantilever reinforced concrete (RC) column and generates a comprehensive, publication-quality performance dashboard.  
# OpenSeesPy Seismic Behavior Evaluation Dashboard for RC Columns

This repository contains an open-source structural engineering pipeline developed in Python using **OpenSeesPy** and **Matplotlib**. The framework performs an advanced nonlinear cyclic displacement-control analysis of a cantilever reinforced concrete (RC) column and generates a comprehensive, publication-quality performance dashboard.

The tool bridges the gap between advanced finite element simulation and fundamental structural mechanics by overlaying a first-principles plastic mechanism check directly over the simulated hysteretic output.

---

## 📊 Dashboard Preview

![Seismic Behavior Evaluation Dashboard](Figure_1.png)

---

## 🏛️ Theoretical Foundations

The predictive capability of this framework is grounded in non-linear structural mechanics, explicitly separating cross-sectional material constitutive laws from macro-element kinematic behavior.

### 1. Constitutive Material Relationships
* **Confined Core Concrete:** Modeled using the Kent-Scott-Park paradigm with degraded linear unloading/reloading stiffening behavior (`Concrete02`). Material parameters account for structural confinement effects, elevating compressive strength to $f_{cc} = -48\text{ MPa}$ and ultimate strain capacity to $\varepsilon_{cu} = -0.0125$.
* **Unconfined Cover Concrete:** Modeled using unconfined parameters ($f_c = -40\text{ MPa}$, $\varepsilon_{cu} = -0.005$) to simulate cover spalling under high cyclic compressive strain gradients.
* **Longitudinal Reinforcement:** Modeled via the Menegotto-Pinto hysteretic formulation (`Steel02`) to accurately capture Bauschinger effects, isotropic strain hardening, and plastic energy dissipation paths.

### 2. First-Principles Verification (Plastic Analysis)
To validate the mathematical integrity of the OpenSees finite element formulation, the script computes the ultimate theoretical shear limit based on a classic plastic flexural mechanism. 

The ultimate plastic moment capacity $M_p$ is determined by the equilibrium couple between the longitudinal tensile steel reinforcement and the concurrent axial load contribution:

$$M_p = M_{\text{steel}} + M_{\text{axial}}$$

$$M_{\text{steel}} = A_{st} \cdot f_y \cdot d'$$

$$M_{\text{axial}} = P \cdot \left(\frac{B}{2} - \frac{d_c}{2}\right)$$

Where:
* $A_{st}$ is the effective total tension steel area ($3 \times \Phi 20$ bars).
* $f_y$ is the steel yield strength (**400 MPa**).
* $d'$ is the distance between opposing rebar centroids (**320 mm**).
* $P$ is the applied invariant axial gravity load (**640 kN**, corresponding to a **10%** axial load ratio).
* $B$ is the column cross-sectional width (**400 mm**).
* $d_c$ is the concrete clear cover (**40 mm**).

Assuming a pure plastic flexural hinge forms at the fixed cantilever base, the theoretical upper-bound shear capacity $V_n$ is dictated by virtual work and static equilibrium:

$$V_n = \pm \frac{M_p}{H}$$

where $H$ represents the clean shear span of the cantilever column (**3000 mm**). This rigid boundary threshold is overlaid on the simulated results as a benchmark for validation.

---

## 🛠️ Dashboard Features & Layout Controls

The visual production engine uses a multi-row grid architecture (`add_gridspec`) deliberately engineered to completely eliminate layout collisions, text clipping, and dead space.

* **Subplot 1: Boundary Kinematics:** A visual schematic of the cantilever boundary state. The lateral force label text is dynamically anchored to the left of the actuator vector tail to guarantee zero spatial collision with the column shaft.
* **Subplot 2: Cross-Section Detail:** A discretized map of the fiber patch coordinates. The material legend is shifted to a dedicated bottom margin to provide an unobstructed view of the cross-section axes.
* **Subplot 3: Lateral Force-Drift Hysteresis:** Real-time extraction of base shear versus drift ratio percentage, benchmarked against the dashed theoretical limits ($\pm V_n$) and target drift limits (**3.0%**).
* **Row 2 Data Panel:** A structural calculations summary block embedded inside a clean layout matrix, allowing the figure to scale seamlessly across different window sizes.

---

## 💻 Technical Specifications & Input Deck

| Structural Property | Design Value | Code Context / Unit |
| :--- | :--- | :--- |
| Column Height ($H$) | 3000 mm | Cantilever Aspect Ratio = 7.5 |
| Section Dimensions ($B \times D$) | 400 mm $\times$ 400 mm | Square Frame Element |
| Concrete Strength ($f'_c$) | 40 MPa | Normal Weight C40 |
| Steel Reinforcement | 8 $\times$ $\Phi$20 Bars | $A_s = 2513.3\text{ mm}^2$ ($\rho_s = 1.57\%$) |
| Axial Load Ratio ($n$) | 0.10 | $P = 640\text{ kN}$ constant gravity |
| Cyclic Drift Protocol | 0.5%, 1.0%, 1.5%, 2.0%, 3.0% | 3 Complete Cycles per Drift Level |

---

## 🚀 Getting Started

### Prerequisites
Ensure your Python environment has the following engineering and visualization libraries installed:
```bash
pip install openseespy numpy matplotlib
