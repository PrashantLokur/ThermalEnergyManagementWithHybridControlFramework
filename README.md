# Constrained Nonlinear MPC for Energy-Optimal Thermal Management of Heat-Pump BEVs

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023a%2B-blue)](https://www.mathworks.com/products/matlab.html)
[![CasADi](https://img.shields.io/badge/CasADi-3.6%2B-green)](https://web.casadi.org/)
[![CoolProp](https://img.shields.io/badge/CoolProp-6.x-orange)](http://www.coolprop.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

**Paper:** *Constrained Nonlinear Model Predictive Control for Energy-Optimal Thermal Management of Heat-Pump BEVs*  
**Target:** IEEE Open Journal of Vehicular Technology (OJVT)  
**Affiliation:** Chalmers University of Technology / Geely Technology Europe (Zeekr)

---

## Overview

This repository contains the MATLAB/CasADi implementation of a Constrained Nonlinear MPC (NMPC) framework for energy-optimal thermal management of Battery Electric Vehicles (BEVs) equipped with a heat-pump system.

The controller simultaneously manages:
- Cabin temperature comfort (heating/cooling)
- Battery thermal conditioning
- Motor and inverter cooling
- Compressor, pump, blower, and radiator fan actuation

The NMPC is formulated as a Nonlinear Program (NLP) solved at each sampling step using IPOPT with an exact Hessian.

---

## Repository Structure

```
├── MPCController_stepNew.m           % Simulink wrapper: unpacks bus, calls core, reports timing
├── MPCController_step_coreLQRNew.m   % NMPC core: NLP build, solve, warm-start cache
├── create_dynamics_nmpcNew.m         % CasADi wrapper: creates symbolic F(x,u,p) function
├── dynamics_mexfreeNew.m             % Physics model: all thermal submodels (CasADi-compatible)
├── estimateParameters.m              % CoolProp parameter estimation at each MPC step
├── thermalLogicAndEstimation.m       % Supervisory layer: mode selection, EXV commands
├── prepParaVec.m                     % Interpolates tuned parameters p1..p11 from data
├── RunThermalComparison.m            % Automated comparison: NMPC vs rule-based, multi-scenario
└── README.md
```

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  Simulink Plant (Simscape)                                      │
│  States x [9×1]: motT, invT, dcdcT, SOC, batT,                 │
│                  p_in, p_out, cabIntT, cabAirT                  │
└──────────────┬──────────────────────────────────────────────────┘
               │ measurements (every Ts = 5s)
               ▼
┌─────────────────────────────────────────────────────────────────┐
│  MPCController_stepNew.m  (Simulink Interpreted MATLAB Fcn)     │
│  • Unpacks input bus                                            │
│  • Unit conversions (degC→K, MPa→Pa)                           │
│  • Timing statistics and deadline warnings                      │
└──────────────┬──────────────────────────────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────────────────────────────┐
│  MPCController_step_coreLQRNew.m                                │
│  • Build NLP once (persistent): CasADi symbolic graph           │
│  • Per step: pack parameters, warm-start, solve IPOPT           │
│  • Warm-start cache (disk): skips cold-start penalty            │
│  • Soft constraints: IC slack, ramp-rate slack, state slacks    │
└──────┬───────────────────────────────────────────────────────────┘
       │                    │                    │
       ▼                    ▼                    ▼
thermalLogicAnd       estimateParameters    prepParaVec
Estimation.m          .m (CoolProp)         .m (interpolation)
(supervisory layer)   (fluid properties)    (tuned params p1..p11)
       │
       ▼
create_dynamics_nmpcNew.m
       │
       ▼
dynamics_mexfreeNew.m
(CasADi-compatible physics)
```

---

## Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| MATLAB | R2024b | Core platform |
| Simulink + Simscape | R2024b | Plant model |
| CasADi | 3.6+ | Symbolic differentiation, NLP formulation |
| IPOPT | 3.14+ | NLP solver (bundled with CasADi) |
| CoolProp (Python) | 6.x | Refrigerant/fluid properties via `py.CoolProp` |
| HSL MA97 | (optional) | Faster linear solver than MUMPS |

### CoolProp setup
CoolProp is called via MATLAB's Python interface:
```matlab
% Test installation:
py.CoolProp.CoolProp.PropsSI('T', 'P', 2e5, 'Q', 1, 'R134a')
```

### CasADi setup
```matlab
addpath('/path/to/casadi');
import casadi.*
```

---

## State and Input Vectors

### States x [9×1] — all in SI units (K, Pa, -)
| Index | Symbol | Description | Units |
|-------|--------|-------------|-------|
| 1 | motT | Motor temperature | K |
| 2 | invT | Inverter temperature | K |
| 3 | dcdcT | DC/DC converter temperature | K |
| 4 | SOC | Battery state of charge | - |
| 5 | batT | Battery temperature | K |
| 6 | p_in | Refrigerant low-side pressure | Pa |
| 7 | p_out | Refrigerant high-side pressure | Pa |
| 8 | cabIntT | Cabin interior thermal mass temperature | K |
| 9 | cabAirT | Cabin air temperature | K |

### Control inputs u [6×1]
| Index | Symbol | Description | Units |
|-------|--------|-------------|-------|
| 1 | compNrpm | Compressor speed | rpm |
| 2 | cabAirMfIn | Cabin blower mass flow | kg/s |
| 3 | motPumpNrpm | Motor coolant pump speed | rpm |
| 4 | batPumpNrpm | Battery coolant pump speed | rpm |
| 5 | heaterPwr | Auxiliary heater power | W |
| 6 | radFanrpm | Radiator fan speed | rpm |

---

## NMPC Formulation

The NMPC solves at each step:

```
min   Σ_{k=0}^{N-1} [ (x9_k - Tref)² × w_cab
                     + u_k' R_s u_k
                     + Δu_k' Rd_s Δu_k × λ_k
                     + pwr_k / pwrEstMax ]
    + (xN - xT)' P (xN - xT)                    (terminal cost)
    + ρ_dyn Σ ||Sdyn_k||²                        (dynamics slack)
    + ρ_ic  ||Sic||²                             (IC slack)
    + w_du  Σ ||Sdu_k||²                         (ramp-rate slack)
    + slack penalties for algebraic constraints

s.t.  x_{k+1} = Φ(x_k, u_k, p_k)               (RK4 discrete dynamics)
      x_1 = x_meas + Sic                         (soft IC)
      |Δu_k| ≤ Δu_max + Sdu_k                   (soft ramp rate)
      p_out_k ≥ p_in_k × 1.3  (soft)            (pressure ratio)
      clntT ∈ [clntLow, clntHigh]  (soft)        (coolant bounds)
      superheat ∈ [-2, 10] K  (soft)             (refrigerant quality)
      u ∈ [umin, umax]                            (actuator bounds)
      x ∈ [xmin, xmax]  (soft via Sx_lo/hi)      (state bounds)
```

**Solver:** IPOPT with exact Hessian, MUMPS/MA97 linear solver  
**Horizon:** N = 30 steps × Ts = 5s = 150s prediction window  
**Sample time:** Ts = 5s (justified by thermal time constants τ ≈ 60-120s)

---

## Key Design Decisions

### Why exact Hessian (not L-BFGS)
The system has strongly coupled nonlinear refrigerant dynamics. L-BFGS fails because:
- The cost has near-zero curvature in 8 of 9 state directions
- Constraint Hessians from refrigerant saturation curves are large
- Slack variable structure causes rapid gradient direction changes

### Soft IC constraint (`Sic_sym`)
The initial condition equality constraint uses a slack variable to prevent infeasibility when plant states are outside NLP bounds (e.g., `p_in ≈ p_out` at cold start). The penalty `ρ_ic = 1e6` ensures the slack is near-zero during normal operation.

### Soft ramp-rate constraint (`Sdu_sym`)
The compressor ramp-rate limit `Δu_max` is enforced via a slack `Sdu ≥ 0`:
```
dU + Sdu ≥ -Δu_max
dU - Sdu ≤ +Δu_max
```
This allows large compressor steps at cold start (when needed for pressure build-up) with a penalty, preventing infeasibility.

### Warm-start cache
After the first successful high-quality solve, the optimal primal/dual solution is saved to `nmpc_warm_cache.mat`. Subsequent runs load this cache to skip the expensive cold-start. The cache is validated by checking `n_w`, `n_g`, `N`, `nu`, `nx`, `np` — any structural change invalidates it automatically.

### Terminal cost
The terminal cost matrix `P` is computed via DARE on the linearized system at the terminal operating point. A 5-level fallback chain (standard DARE → discounted DARE → finite-horizon Riccati → Lyapunov → scaled identity) ensures robustness. Terminal data is cached and only recomputed when ambient temperature, reference, or parameters shift significantly.

---

## Quick Start

### 1. Setup
```matlab
% In MATLAB:
addpath(genpath('path/to/casadi'));
addpath(genpath('path/to/this/repo'));


```

### 2. Run comparison
```matlab
% Interactive menu:
RunThermalComparison()

% Or directly:
RunThermalComparison('both', [-5, -7, -10])   % NMPC vs rule-based at 3 temperatures
RunThermalComparison('nmpc', -10)             % NMPC only at -10°C
```

### 3. Reset between runs
```matlab
% Required when changing N, Ts, bounds, or model structure:
clear MPCController_step_coreLQRNew
delete('nmpc_warm_cache.mat')
```

---

## Scenarios

| Scenario | Ambient | Initial T | Description |
|----------|---------|-----------|-------------|
| cold_weather | -10°C | -10°C | Cold start, heat pump active |
| cold_weather | -7°C | -7°C | Mild cold, heat pump |
| cold_weather | -5°C | -5°C | Near threshold |
---

## IPOPT Configuration

Key settings (in `MPCController_step_coreLQRNew.m`):

```matlab
opts.ipopt.hessian_approximation = 'exact';    % mandatory
opts.ipopt.linear_solver         = 'ma97';     % or 'mumps'
opts.ipopt.max_iter              = 1000;
opts.ipopt.tol                   = 1e-3;
opts.ipopt.acceptable_tol        = 1e-2;
opts.ipopt.acceptable_iter       = 5;          % early exit on good iterates
```

Set `DEBUG_IPOPT = true` inside the build block to enable detailed IPOPT output to `ipopt_debug.txt`.

---

## Known Issues and Limitations

1. **Solve time** — Mean solve time approx 450ms at N=30, Ts=1s on a desktop CPU. Suitable for supervisory-level control. Embedded deployment requires further optimization or reduced horizon.

2. **CoolProp calls** — `estimateParameters.m` makes ~40 CoolProp calls per MPC step. These are the main overhead source (~150-400ms). Pre-tabulation would reduce this significantly.

3. **First simulation step** — The NLP build (CasADi symbolic graph + IPOPT compilation) takes 5-15 minutes on first run. Subsequent runs reuse the persistent solver object within the same MATLAB session.

4. **Pressure ratio at cold start** — When `p_in ≈ p_out` (system not pressurized), a pressure injection heuristic is used for parameter computation. The IC slack handles the NLP initial condition.

---

## File-by-File Description

### `MPCController_stepNew.m`
Simulink **Interpreted MATLAB Function** block wrapper.
- Reads workspace parameters `N`, `Ts`, `nx`, `nu`, `N_MAX`
- Unpacks the input bus (states, speed preview, heat loads, setpoint)
- Unit conversions: temperatures degC→K, pressures MPa→Pa
- Calls `MPCController_step_coreLQRNew`
- Reports timing statistics and deadline warnings every 10 steps
- **Sample time:** Set to `Ts` in Simulink block parameters

### `MPCController_step_coreLQRNew.m`
NMPC core function — the main contribution.
- Builds the NLP once using CasADi symbolic differentiation (persistent)
- Packs the stage parameter matrix `Pk` using CoolProp-estimated properties
- Warm-starts from shifted previous solution or loaded cache
- Solves with IPOPT; handles infeasibility gracefully
- Saves first high-quality solution to `nmpc_warm_cache.mat`

### `create_dynamics_nmpcNew.m`
Creates the CasADi function `F(x, u, p) → [xdot, y, pwr]`.
- Defines decision input ordering (`inputSignal`)
- Defines parameter ordering (`estimatedParams`, `controlOut`, `tunePara`)
- Returns `packParams` function handle for packing numeric parameter vectors

### `dynamics_mexfreeNew.m`
Physics model — all thermal subsystems implemented as CasADi-compatible MATLAB functions.
- Compressor, pumps, heat exchangers, refrigerant circuit, cabin, battery
- All divisions protected with `+ 1e-6` denominators
- Sigmoid gates replace discontinuous on/off logic
- Air mass flow: `airMf = max(ram_air + fan_air, airMf_min)` ← floor, not ceiling
- Fan lag: `alpha = exp(-dt_plant / tau_fan)` using actual plant timestep

### `estimateParameters.m`
Called each MPC step; computes fluid properties via CoolProp.
- Refrigerant saturation properties at current p_in, p_out
- Finite-difference thermodynamic derivatives for pressure dynamics
- Ambient air properties using actual `input.envT` (not hardcoded -10°C)
- **Note:** `estParams` local variable (not `estimatedParams`) used for low-side properties — verified correct assignment downstream

### `thermalLogicAndEstimation.m`
Supervisory layer — runs independently of NMPC at each step.
- Mode selection: heat pump (mode=2), cooling (mode=1), off (mode=0)
- EXV commands, parallel/serial coolant switching
- Heater hysteresis with correct asymmetric thresholds
- Calls `estimateParameters` for fluid property estimation

### `prepParaVec.m`
Interpolates 11 tuned scalar parameters `p1..p11` from calibration data.
- Uses `scatteredInterpolant` over (ambient temperature, state) space
- Persistent cache: interpolants rebuilt only when data changes
- Fallback to nearest-neighbour if interpolated value is non-finite

### `RunThermalComparison.m`
Automated multi-scenario comparison script.
- Switches controller via Constant block `set_param`
- Sets cold-weather scenario parameters in Simulink model workspace
- Runs simulation, extracts energy from `simlog`
- Prints per-component energy table and NMPC vs rule-based comparison
- **Update required:** `constant_block` path and `simlog` field paths for your model

---

## Energy Accounting

Energy is computed from Simscape `simlog` after simulation:
```matlab
stats = compute_energy_results(simlog_PlantMdlMain, 'PlantMdlMain');
compare_energy(stats_base, stats_nmpc);
```

Components tracked: compressor, cabin blower, motor pump, battery pump, auxiliary heater, radiator fan.
Mechanical → electrical conversion uses efficiencies from `pwrEstimation` in `dynamics_mexfreeNew.m`.

---

## Citation

If you use this code, please cite:
```
@article{lokur2025nmpc,
  title   = {Constrained Nonlinear Model Predictive Control for Energy-Optimal
             Thermal Management of Heat-Pump BEVs},
  author  = {Lokur, Prashant and Murgovski, Nikolce},
  journal = {IEEE Open Journal of Vehicular Technology},
  year    = {2025},
  note    = {Under review}
}
```

---

## License

MIT License — see `LICENSE` file.  
© 2025 Prashant Lokur, Chalmers University of Technology / Geely Technology Europe (Zeekr)
