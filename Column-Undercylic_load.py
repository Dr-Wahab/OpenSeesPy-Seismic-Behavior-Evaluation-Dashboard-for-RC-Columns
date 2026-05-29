import openseespy.opensees as ops
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ==========================================================
# 1. INITIALIZATION & GEOMETRY BOUNDS (SI Units: N, mm)
# ==========================================================
ops.wipe()
ops.model('basic', '-ndm', 2, '-ndf', 3) 

H_col = 3000.0       # 3000 mm
B_col = 400.0        # 400 mm
D_col = 400.0        # 400 mm
Ag = B_col * D_col   # 160,000 mm^2

ops.node(1, 0.0, 0.0)    
ops.node(2, 0.0, H_col)  
ops.fix(1, 1, 1, 1)

# ==========================================================
# 2. MATERIAL CONSTITUTIVE LAWS (N/mm^2 == MPa)
# ==========================================================
fcc = -48.0          # Confined Core Concrete
ecc0 = -0.004        
fccu = -10.0         
eccu = -0.0125       
ft = 4.0             
Ets = 4000.0         
ops.uniaxialMaterial('Concrete02', 1, fcc, ecc0, fccu, eccu, 0.1, ft, Ets)

fc = -40.0           # Unconfined Cover Concrete
ec0 = -0.002         
fcu = 0.0           
ecu = -0.005         
ops.uniaxialMaterial('Concrete02', 2, fc, ec0, fcu, ecu, 0.1, ft, Ets)

fy = 400.0           # Steel Grade HB400
Es = 200000.0        
b_ratio = 0.015       
R0, cR1, cR2 = 18.0, 0.925, 0.15 
ops.uniaxialMaterial('Steel02', 3, fy, Es, b_ratio, R0, cR1, cR2)

# ==========================================================
# 3. FIBER SECTION GENERATION (Discretized in mm)
# ==========================================================
sec_tag = 1
cover = 40.0        
bar_area = np.pi * (20.0**2) / 4.0  

core_Y = (B_col / 2.0) - cover   
core_Z = (D_col / 2.0) - cover   

ops.section('Fiber', sec_tag)
# Confined Core
ops.patch('quad', 1, 16, 16, -core_Y, -core_Z, core_Y, -core_Z, core_Y, core_Z, -core_Y, core_Z)

# Unconfined Cover Patches
ops.patch('quad', 2, 4, 16, -B_col/2, -D_col/2,  core_Y, -D_col/2,  core_Y, -core_Z, -B_col/2, -core_Z)
ops.patch('quad', 2, 4, 16, -core_Y,  core_Z,   B_col/2,  core_Z,   B_col/2,  D_col/2, -core_Y,  D_col/2)
ops.patch('quad', 2, 16, 4, -B_col/2, -core_Z, -core_Y, -core_Z, -core_Y,  core_Z, -B_col/2,  core_Z)
ops.patch('quad', 2, 16, 4,  core_Y,  -core_Z,  B_col/2, -core_Z,  B_col/2,  core_Z,  core_Y,   core_Z) # <-- Fixed auto_Y to core_Y

# Longitudinal Reinforcement (8d20)
ops.fiber( core_Y,  core_Z, bar_area, 3)
ops.fiber( core_Y,     0.0, bar_area, 3)
ops.fiber( core_Y, -core_Z, bar_area, 3)
ops.fiber(    0.0,  core_Z, bar_area, 3)
ops.fiber(    0.0, -core_Z, bar_area, 3)
ops.fiber(-core_Y,  core_Z, bar_area, 3)
ops.fiber(-core_Y,     0.0, bar_area, 3)
ops.fiber(-core_Y, -core_Z, bar_area, 3)

ops.geomTransf('Linear', 1)
ops.element('nonlinearBeamColumn', 1, 1, 2, 5, sec_tag, 1) 

# ==========================================================
# 4. GRAVITY LOADING BALANCING
# ==========================================================
P_axial = -0.10 * 40.0 * Ag  
ops.timeSeries('Linear', 1)
ops.pattern('Plain', 1, 1)
ops.load(2, 0.0, P_axial, 0.0)

ops.constraints('Transformation')
ops.numberer('RCM')
ops.system('BandGeneral')
ops.test('NormDispIncr', 1.0e-6, 30)
ops.algorithm('Newton')
ops.integrator('LoadControl', 0.1)
ops.analysis('Static')
ops.analyze(10)
ops.loadConst('-time', 0.0)

# ==========================================================
# 5. CYCLIC ENGINE (3 CYCLES PER DRIFT LEVEL)
# ==========================================================
drift_protocols = [0.005, 0.010, 0.015, 0.020, 0.030] 
cycles_per_level = 3
step_size = 0.00025                                   

ops.pattern('Plain', 2, 1)
ops.load(2, 1.0, 0.0, 0.0) 

all_drifts = []
all_shears = []

for drift in drift_protocols:
    peak_disp = drift * H_col 
    for cycle in range(cycles_per_level):
        ops.integrator('DisplacementControl', 2, 1, step_size * H_col)
        ops.analysis('Static')
        while ops.nodeDisp(2, 1) < peak_disp:
            if ops.analyze(1) != 0: break
            all_drifts.append(ops.nodeDisp(2, 1) / H_col * 100)
            ops.reactions()
            all_shears.append(-ops.nodeReaction(1, 1) / 1000) 
            
        ops.integrator('DisplacementControl', 2, 1, -step_size * H_col)
        while ops.nodeDisp(2, 1) > -peak_disp:
            if ops.analyze(1) != 0: break
            all_drifts.append(ops.nodeDisp(2, 1) / H_col * 100)
            ops.reactions()
            all_shears.append(-ops.nodeReaction(1, 1) / 1000)
            
        ops.integrator('DisplacementControl', 2, 1, step_size * H_col)
        while ops.nodeDisp(2, 1) < 0.0:
            if ops.analyze(1) != 0: break
            all_drifts.append(ops.nodeDisp(2, 1) / H_col * 100)
            ops.reactions()
            all_shears.append(-ops.nodeReaction(1, 1) / 1000)

# ==========================================================
# 6. STRUCTURAL CAPACITY CALCULATIONS
# ==========================================================
Ast_tension = 3.0 * bar_area  
d_prime = 2.0 * core_Y        
M_steel = (Ast_tension * fy * d_prime) / 1.0e6  
M_axial = (abs(P_axial) * (B_col / 2.0 - cover / 2.0)) / 1.0e6  
Mp_theoretical = M_steel + M_axial              
Vn_theoretical = Mp_theoretical / (H_col / 1000.0) 

# ==========================================================
# 7. HIGH-PRODUCTION VISUALIZATION ENGINE (GRID ANCHORED)
# ==========================================================
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.size'] = 9.5

fig = plt.figure(figsize=(16, 8.8))
# Explicit 2-row layout isolates the graphs from the lower summary block
gs = fig.add_gridspec(2, 3, width_ratios=[1.0, 1.2, 1.8], height_ratios=[4.2, 1.0])

# --- Subplot 1: Boundary Kinematics (No Overlaps) ---
ax_schem = fig.add_subplot(gs[0, 0])
ax_schem.plot([0, 0], [0, H_col], color='#4b5563', linewidth=12, solid_capstyle='butt', zorder=1)
ax_schem.plot([-350, 350], [0, 0], color='#111111', linewidth=5, zorder=2)
for k in np.linspace(-300, 300, 7):
    ax_schem.plot([k, k-50], [0, -90], color='#111111', linewidth=1.5)

# Axial Load Vector
ax_schem.annotate('', xy=(0, H_col), xytext=(0, H_col+450), arrowprops=dict(facecolor='#dc2626', edgecolor='#dc2626', width=3, headwidth=9))
ax_schem.text(80, H_col+220, f'Axial Load P\n640 kN\n(10% $A_g f\'_c$)', color='#dc2626', weight='bold', fontsize=9)

# Lateral Actuator Arrow & Text (Shifted entirely to the left quadrant)
ax_schem.annotate('', xy=(0, H_col), xytext=(-600, H_col), arrowprops=dict(facecolor='#1e40af', edgecolor='#1e40af', width=2.5, headwidth=7))
ax_schem.text(-620, H_col, 'Cyclic Lateral\nPush (3% Drift)', color='#1e40af', weight='bold', fontsize=9, ha='right', va='center')

ax_schem.set_xlim([-950, 950])
ax_schem.set_ylim([-300, H_col + 750])
ax_schem.set_title("Column Boundary Conditions", fontsize=11, fontweight='bold', pad=12)
ax_schem.axis('off')

# --- Subplot 2: Cross Section Detail (Clean Horizontal Axis Footer) ---
ax_sec = fig.add_subplot(gs[0, 1])
ax_sec.add_patch(patches.Rectangle((-200, -200), 400, 400, facecolor='#f8fafc', edgecolor='#0f172a', linewidth=1.8, label='Cover Concrete (Plain C40)'))
ax_sec.add_patch(patches.Rectangle((-160, -160), 320, 320, facecolor='#cbd5e1', edgecolor='#991b1b', linestyle='--', linewidth=1.8, label='Confined Core (Mander C40)'))
bar_coords = [(-160,-160), (0,-160), (160,-160), (-160,0), (160,0), (-160,160), (0,160), (160,160)]
for idx, (by, bz) in enumerate(bar_coords):
    ax_sec.add_patch(patches.Circle((by, bz), 16, facecolor='#0f172a', zorder=4, label='Longitudinal Rebar (8d20 HB400)' if idx == 0 else ""))
ax_sec.add_patch(patches.Rectangle((-166, -166), 332, 332, facecolor='none', edgecolor='#334155', linewidth=1.2, label='Stirrup Ties (d8 @ 150mm)'))

ax_sec.set_xlim([-250, 250])
ax_sec.set_ylim([-250, 250])
ax_sec.set_aspect('equal')
ax_sec.set_title("Cross-Section Detail", fontsize=11, fontweight='bold', pad=12)
ax_sec.grid(True, linestyle=':', alpha=0.5)
ax_sec.set_xlabel("Width B (mm)", fontsize=9, fontweight='bold')
ax_sec.set_ylabel("Depth D (mm)", fontsize=9, fontweight='bold')

# Legend positioned cleanly underneath the plot, leaving side areas completely open
ax_sec.legend(loc='upper center', bbox_to_anchor=(0.5, -0.24), ncol=1, fontsize=8, framealpha=1.0, edgecolor='#cbd5e1')

# --- Subplot 3: Hysteresis Performance Curve ---
ax_curve = fig.add_subplot(gs[0, 2])
drifts = np.array(all_drifts)
shears = np.array(all_shears)
ax_curve.plot(drifts, shears, color='#1e40af', linewidth=1.4, alpha=0.9, label='OpenSees Fiber Simulation')
ax_curve.axhline(Vn_theoretical, color='#dc2626', linestyle='--', linewidth=1.8, label=f'Theoretical Limit $V_n$ = $\pm${Vn_theoretical:.1f} kN')
ax_curve.axhline(-Vn_theoretical, color='#dc2626', linestyle='--', linewidth=1.8)
ax_curve.axhline(0, color='#64748b', linewidth=0.8, alpha=0.5)
ax_curve.axvline(0, color='#64748b', linewidth=0.8, alpha=0.5)
ax_curve.axvline(3.0, color='#475569', linestyle=':', linewidth=1.5, label='3.0% Target Drift')
ax_curve.axvline(-3.0, color='#475569', linestyle=':', linewidth=1.5)
ax_curve.set_title("Lateral Force-Drift Performance Profile", fontsize=11, fontweight='bold', pad=12)
ax_curve.set_xlabel("Lateral Displacement Drift Ratio (%)", fontweight='bold', fontsize=9)
ax_curve.set_ylabel("Resisting Base Shear Force $V_b$ (kN)", fontweight='bold', fontsize=9)
ax_curve.grid(True, linestyle='--', alpha=0.5)
ax_curve.set_xlim([-3.5, 3.5])
ax_curve.set_ylim([-100, 100])
ax_curve.legend(loc='upper left', fontsize=8.5, framealpha=1.0)

# --- Row 2: Isolated Calculations Panel (Zero Intersect Design) ---
ax_summary = fig.add_subplot(gs[1, :])
ax_summary.axis('off')  

summary_text = (
    "======================================= STRUCTURAL ANALYSIS SUMMARY =======================================\n"
    f"  • Geometry: H = {H_col:.0f} mm, Section = {B_col:.0f}x{D_col:.0f} mm  |  Materials: Concrete C40 (f'c = 40 MPa), Steel HB400 (fy = 400 MPa)\n"
    f"  • Reinforcement Setup: 8d20 Longitudinal Configuration (As = {8*bar_area:.1f} mm²), Transverse Ties = d8 @ 150 mm Spacing\n"
    f"  • First-Principles Mathematical Calculations:\n"
    f"      M_steel = Ast_tension * fy * d' = {Ast_tension:.1f} * 400 * {d_prime:.0f} = {M_steel:.1f} kN·m   |   M_axial = P * (h/2 - dc/2) = 640 * 180 = {M_axial:.1f} kN·m\n"
    f"      M_p = M_steel + M_axial = {Mp_theoretical:.1f} kN·m   =====>   Plastic Shear Resistance Limit: V_n = M_p / H = {Vn_theoretical:.1f} kN"
)

# Render compact text card with a sleek slate background and steel blue border
ax_summary.text(0.5, 0.4, summary_text, fontsize=8.5, family='monospace', weight='bold',
                color='#0f172a', bbox=dict(boxstyle='round,pad=0.5', facecolor='#f8fafc', edgecolor='#1e40af', linewidth=1.5, alpha=1.0),
                horizontalalignment='center', verticalalignment='center')

plt.suptitle("Seismic Behavior Evaluation Dashboard: Reinforced Concrete Column Framework", fontsize=13, fontweight='bold', y=0.96)

# Controlled vertical packing engine parameters (wspace & hspace) lock positions tightly
plt.subplots_adjust(left=0.06, right=0.96, top=0.88, bottom=0.05, wspace=0.32, hspace=0.48)
plt.show()