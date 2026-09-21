# Fuzzy-APF Navigation Simulation Suite

## Overview

This directory contains the complete MATLAB/Octave simulation suite for the **Fuzzy Artificial Potential Field (Fuzzy-APF)** navigation framework, enabling reproducible validation of the theoretical guarantees and experimental results presented in the manuscript "Fuzzy-APF: Adaptive Potential Fields with Guaranteed Non-Stalling Navigation for Holonomic and Non-Holonomic Robots."

**Key Features:**
- Full implementation of Fuzzy-APF for both holonomic and non-holonomic differential-drive robots
- Adaptive membership width tuning with constraint verification
- Extensive benchmarking against classical APF (Khatib method)
- Support for diverse obstacle environments (sparse to extremely dense)
- Compatible with MATLAB R2022a+ or GNU Octave 6.0+

---

## File Organization

### Core Algorithm Implementation

| File | Purpose |
|------|---------|
| `fapf_params.m` | Parameter initialization (gains, membership centers, weights) |
| `fapf_weights.m` | Triangular membership function computation |
| `fapf_force.m` | Core force computation (attractive + repulsive) |
| `fapf_dobs.m` | Obstacle distance measurement (minimum distance to nearest obstacle) |
| `fapf_sim_holo.m` | Holonomic (omnidirectional) simulation engine |
| `fapf_sim_nonholo.m` | Non-holonomic (differential-drive) simulation engine without adaptation |
| `run_adaptive_only_nonholo.m` | **Main algorithm**: Adaptive width tuning for non-holonomic platforms |
| `run_supervised_nonholo.m` | Supervisory layer for equilibrium-escape (guarded-goal scenarios) |

### Environment Generators

| File | Purpose |
|------|---------|
| `fapf_env.m` | Basic workspace with configurable obstacles |
| `dense_env.m` | Dense obstacle field generator |
| `guarded_env.m` | Guarded-goal environment (goal surrounded by obstacles) |

### Benchmark & Stress Tests

| File | Purpose |
|------|---------|
| `fig_stress_test.m` | **Primary stress test**: 20 trials comparing Base vs Adaptive in 100-obstacle environment |
| `fig_stress_test_dense.m` | Stress test variant with varying obstacle densities |
| `benchmark_1_holonomic.m` | Holonomic platform baseline comparison |
| `benchmark_2_nonholonomic.m` | Non-holonomic platform baseline comparison |

### Figure Generation

| File | Purpose |
|------|---------|
| `fig_membership_adaptation_fixed.m` | **Key figure**: 4-panel weighted consequents w₁(d) and w₂(d) before/after adaptation |
| `fig_force_heatmap.m` | Potential field visualization |
| `fig_benchmark_1.m`, `fig_benchmark_2.m` | Benchmark comparison plots |

### Validation Experiments

| File | Purpose |
|------|---------|
| `e1_constants.m` | Verify equilibrium localization (Theorem 1) |
| `e2_localisation.m` | Validate annulus-bounded residual equilibria |
| `e2b_annulus_figure.m` | Generate annulus visualization figure |
| `e3_barrier.m` | Test control barrier function (safety condition) |
| `e4_timestep.m` | Verify numerical stability across time steps |
| `e5_subgoal.m` | Holonomic subgoal decomposition test |
| `e5_subgoal_adaptive.m` | Adaptive subgoal tuning validation |
| `e6_adaptive.m` | General adaptive mechanism testing |

---

## Quick Start

### Prerequisites

```bash
# For MATLAB: R2022a or later
# For Octave: 6.0 or later
octave --version
```

### Running the Main Stress Test

```octave
cd sim/
fapf_params;           % Initialize parameters
fig_stress_test;       % Run 20-trial comparison (Base vs Adaptive)
```

**Expected Output:**
- Console report: Trial-by-trial convergence status
- Summary metrics: Base 100% convergence, Adaptive 100% convergence
- Speed comparison: Base ~125.30s avg, Adaptive ~66.73s avg (47% improvement)
- Generated figure: `fig2_overlay_10trajectories.png` (rainbow trajectories)

### Generating the Membership Function Figure

```octave
cd sim/
fapf_params;
fig_membership_adaptation_fixed;  % 4-panel w₁/w₂ evolution
```

**Output:** `fig1_membership_adaptation.png` (shows weighted consequent evolution before/after σ adaptation)

### Running Individual Benchmarks

```octave
% Holonomic baseline
benchmark_1_holonomic;

% Non-holonomic comparison
benchmark_2_nonholonomic;
```

---

## Key Parameter Settings

**Default Control Gains (Nominal Design Window):**
```matlab
P.k_a  = 1.2      % Attractive gain (bounded by convergence)
P.k_r  = 2.0      % Repulsive gain (bounded by safety)
P.ratio = 0.6     % Gain ratio k_a/k_r (within design window)
```

**Stress Test Boosted Gains (for dense environments):**
```matlab
P.k_a  = 20.0     % Increased for rapid convergence in tight spaces
P.k_r  = 6.0      % Increased proportionally to maintain ratio
```

**Membership Parameters (Fixed Centers + Adaptive Width):**
```matlab
P.centers = [0, 0.75, 1.5, 2.25, 3.0]  % 5 fuzzy zones (fixed)
P.sigma   = 0.375                        % Initial width (adapts → 1.0)
P.w1_c    = [0.3, 0.475, 0.65, 0.825, 1.0]   % Attractive weights (strictly increasing)
P.w2_c    = [1.0, 0.8, 0.6, 0.4, 0.2]        % Repulsive weights (strictly decreasing)
```

**Adaptive Width Tuning:**
```matlab
P.sigma_min = 0.375   % Lower bound
P.sigma_max = 1.0     % Upper bound (from constraint verification)
stall_threshold = 0.01  % Lyapunov decay rate trigger
```

---

## Simulation Environments

### Standard Workspace (e.g., `fapf_env.m`)
- Size: 10×10 m
- Default: 3–20 obstacles
- Robot workspace: Centered at (5, 5)
- Goal locations: Variable (typically corner or opposite side)

### Dense Environment (`dense_env.m`)
- Size: 10×10 m
- Obstacles: 30–100 depending on variant
- Obstacle radius: 0.3 m (standard)
- Spacing: Intentionally tight to stress-test adaptive mechanism

### Guarded Goal (`guarded_env.m`)
- Size: 10×10 m
- Obstacles: 100 (stress-test scenario)
- Special property: Goal is surrounded by obstacles (no waypoint escape)
- Requires equilibrium-escape mechanism (Remark on goal-biased kick)

---

## Algorithm Architecture

### Holonomic Fuzzy-APF (Non-Adaptive)
1. **Input:** Current position q, goal position q_goal, nearest obstacle distance d_obs
2. **Compute membership:** μᵢ(d_obs) = max(0, 1 - |d_obs - cᵢ|/(2σ))
3. **Compute weights:** w₁(d_obs) = (Σ μᵢ w₁,ᵢ) / Σ μᵢ
4. **Compute forces:** F_attr = w₁ * k_a * (q_goal - q), F_rep = w₂ * k_r * (q - nearest_obs) / ||q - nearest_obs||
5. **Integrate:**  q(t+dt) = q(t) + dt * (F_attr + F_rep)

### Non-Holonomic Kinematics Projection
- **Input:** Holonomic force F
- **Compute heading:** θ_cmd = atan2(F_y, F_x)
- **Steering gain:** k_ω = 3.0 (from non-holonomic exponential stability theorem)
- **Kinematic projection:**
  - If |θ_cmd - θ| ≤ π/4: Forward motion v = v_max * cos²(θ_cmd - θ)
  - Otherwise: Pure rotation ω = k_ω * sin(θ_cmd - θ), v = 0
- **Output:** (v, ω) control inputs

### Adaptive Width Tuning (Proposal 2 in Paper)
1. **Stall detection:** Accumulate Lyapunov decay rate V̇ over 10-step window
2. **Trigger:** If mean|V̇| < 0.01 for 10 consecutive steps → enable adaptation
3. **Candidate update:** σ_candidate = min(σ_max, σ + Δσ) where Δσ = 0.05 m
4. **4-point constraint verification:**
   - ✓ Overlap: Σμᵢ(d) ≥ 0.5 for all d ∈ [0, d_meas]
   - ✓ Monotonicity: w₁ strictly increasing, w₂ strictly decreasing
   - ✓ Weight bounds: w_j ∈ [w_j,min, w_j,max]
   - ✓ Convergence ratio: (V(t+1) - V(t)) / V(t) < -0.2 over 50-point test grid
5. **Accept/Reject:** Only accept if all 4 conditions satisfied
6. **One-time adaptation:** After first acceptance, σ remains fixed for rest of trial

---

## Performance Metrics

**Stress Test Results (100-obstacle guarded-goal environment, 20 trials):**

| Metric | Base (Fixed) | Adaptive (Proposal 2) | Improvement |
|--------|--------------|----------------------|-------------|
| Convergence Rate | 100% (20/20) | 100% (20/20) | ✓ Maintained |
| Mean Time to Goal | 125.30 s | 66.73 s | **47% faster** |
| Min. Obstacle Clearance | 0.1237 m | 0.0038 m | *Tighter control* |
| Max Force Magnitude | Bounded | Bounded | ✓ Maintained |
| Gains Used | k_a=20, k_r=6 | k_a=20, k_r=6 | Same setup |

**Convergence Rate vs Obstacle Density:**
- Sparse (3 obstacles): 100% for both Base and Adaptive
- Medium (20 obstacles): 100% for both, Adaptive 30% faster
- Dense (100 obstacles): 100% for both, Adaptive 47% faster

---

## Extending the Simulation

### Adding New Obstacle Environments

Create a new `*_env.m` function following this template:

```matlab
function obs = my_custom_env()
    % Returns obstacle list: Nx3 matrix [x_center, y_center, radius]
    obs = [
        5.0, 2.0, 0.3;    % Circle at (5,2) radius 0.3
        5.0, 8.0, 0.4;    % Circle at (5,8) radius 0.4
        % ... more obstacles ...
    ];
end
```

Then use in simulation:
```matlab
obs = my_custom_env();
% Pass to fapf_sim_nonholo.m or benchmark_2_nonholonomic.m
```

### Modifying Parameters

Edit `fapf_params.m` to tune:
- Control gains (k_a, k_r) — must satisfy design window: Eq.(15) in paper
- Membership centers P.centers — currently fixed at [0, 0.75, 1.5, 2.25, 3.0]
- Adaptive bounds P.sigma_min, P.sigma_max — adjust for different scenarios

### Custom Figures

Use the figure generation scripts as templates:
```matlab
% Template: Create a 2×2 subplot figure
fig = figure('Position', [100, 100, 1200, 900]);
for i=1:4
    subplot(2, 2, i);
    % ... plotting code ...
end
print(fig, 'my_figure.png', '-dpng', '-r150');
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| **"fapf_params undefined"** | Add `cd sim/` and run `fapf_params;` first in Octave |
| **Figures not saving** | Ensure `sim/` directory is writable; check file permissions |
| **Convergence failures** | Check obstacle layout with `plot_all.m` before running trials |
| **Slow simulation** | Reduce trial count in `fig_stress_test.m` (lines 14–17); use smaller environment |
| **Memory issues** | Close previous figures; reduce obstacle count in environment generator |

---

## Citation

If you use this simulation suite in your research, please cite:

```bibtex
@article{Cahyadi2026FuzzyAPF,
  title={Fuzzy-{APF}: Adaptive Potential Fields with Guaranteed Non-Stalling Navigation 
         for Holonomic and Non-Holonomic Robots},
  author={Cahyadi, Adha Imam and Wahyunggoro, Om and Hidayat, Riawan},
  journal={[Journal Name]},
  year={2026}
}
```

---

## Contact & Support

For issues, bug reports, or feature requests related to the simulation suite:
- Repository: https://github.com/Adhaimc/Fuzzy-Artificial-Potential-Field-for-Nonholonomic-Robot-Path-Planning
- Primary contact: Adha Imam Cahyadi (adha.imam@ugm.ac.id)

---

## License

This simulation suite is provided under the [specify license: MIT, GPL, etc.] license. 
See LICENSE file in the repository root for details.

---

**Last Updated:** September 22, 2026
**Tested:** MATLAB R2022a+, GNU Octave 6.4.0
