# Energy-Optimal Thermal Management of Heat-Pump Battery Electric Vehicles

[![MATLAB](https://img.shields.io/badge/MATLAB-R2024b%2B-blue)](https://www.mathworks.com/products/matlab.html)
[![CasADi](https://img.shields.io/badge/CasADi-3.6%2B-green)](https://web.casadi.org/)
[![CoolProp](https://img.shields.io/badge/CoolProp-6.x-orange)](http://www.coolprop.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

This repository contains the MATLAB/CasADi implementation accompanying:

> **P. Lokur and N. Murgovski**, "Energy-Optimal Thermal Management of Heat-Pump Battery Electric Vehicles," submitted to *IEEE Open Journal of Vehicular Technology*, 2026.

The framework coordinates six thermal actuators (compressor, cabin blower, motor pump, battery pump, auxiliary heater, radiator fan) via constrained Nonlinear Model Predictive Control (NMPC) to minimize thermal energy consumption while maintaining cabin comfort and component temperature limits.

---

## Requirements

| Package | Version | Notes |
|---------|---------|-------|
| MATLAB | R2024b+ | With Simulink and Simscape |
| [CasADi](https://web.casadi.org/get/) | 3.6+ | IPOPT bundled |
| [CoolProp](http://www.coolprop.org/) | 6.x | Python package, called via MATLAB's `py` interface |
| HSL MA97 | optional | Faster linear solver; MUMPS is used by default |

**Plant model:** [MathWorks Simscape Electric Vehicle Thermal Management with Heat Pump](https://www.mathworks.com/help/simscape-fluids/ug/electric-vehicle-thermal-management-heat-pump.html) (included with Simscape Fluids).

---

## Setup

1. **Install dependencies** listed above.

2. **Add paths** in MATLAB:
   ```matlab
   addpath(genpath('<path_to_casadi>'));
   addpath(genpath('<path_to_this_repo>'));
   ```

3. **Verify CoolProp:**
   ```matlab
   py.CoolProp.CoolProp.PropsSI('T', 'P', 2e5, 'Q', 1, 'R134a')
   ```

4. **Verify CasADi:**
   ```matlab
   import casadi.*
   casadi.SX.sym('test');
   ```

> **Note:** The first simulation step takes few seconds to minutes to build the CasADi symbolic graph and compile the IPOPT solver. Subsequent steps reuse the persistent solver object.

---

## Usage

```matlab
% Run NMPC vs rule-based comparison at multiple ambient temperatures:
RunThermalComparison('both', [-5, -7, -10])

% NMPC only at a single temperature:
RunThermalComparison('nmpc', -10)
```

To reset the solver (required when changing horizon, sample time, bounds, or model structure):
```matlab
clear MPCController_step_coreLQRNew
delete('nmpc_warm_cache.mat')
```

---

## Repository Structure

| File | Description |
|------|-------------|
| `RunThermalComparison.m` | Entry point: multi-scenario NMPC vs rule-based comparison |
| `MPCController_stepNew.m` | Simulink wrapper: unpacks bus, unit conversions, calls core |
| `MPCController_step_coreLQRNew.m` | NMPC core: NLP construction, IPOPT solve, warm-starting |
| `create_dynamics_nmpcNew.m` | CasADi symbolic dynamics `F(x, u, p)` |
| `dynamics_mexfreeNew.m` | Control-oriented thermal model (CasADi-compatible) |
| `estimateParameters.m` | CoolProp fluid property estimation at each step |
| `thermalLogicAndEstimation.m` | Supervisory layer: mode selection, EXV commands |
| `prepParaVec.m` | Parameter scheduling via interpolation |

---

## NMPC Overview

- **Solver:** IPOPT with exact Hessian
- **Linear solver:** MA97 (or MUMPS)
- **Horizon:** N = 30 steps, Δt = 1 s
- **States (9):** Motor, inverter, DC-DC temperatures; SOC; battery temperature; refrigerant pressures (low/high); cabin interior and air temperatures
- **Inputs (6):** Compressor speed, blower flow, motor pump speed, battery pump speed, heater power, radiator fan speed
- **Terminal cost:** DARE-based with 4-level fallback (discounted DARE → finite Riccati → Lyapunov → scaled identity)
- **Soft constraints:** Initial condition, ramp-rate, preferred state bounds, algebraic constraints (pressure ratio, superheat, coolant limits)

---

## Citation

If you use this code, please cite the accompanying paper:

> P. Lokur and N. Murgovski, "Energy-Optimal Thermal Management of Heat-Pump Battery Electric Vehicles," submitted to *IEEE Open Journal of Vehicular Technology*, 2026.

A BibTeX entry will be provided upon publication.

## License

MIT License — see [LICENSE](LICENSE).