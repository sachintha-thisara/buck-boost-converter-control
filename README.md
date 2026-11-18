# Dynamic Modeling and Cascade PI Control of Non-Linear Buck-Boost DC-DC Converter

![MATLAB](https://img.shields.io/badge/MATLAB-R2024b-blue.svg)
![Power Electronics](https://img.shields.io/badge/Domain-Power%20Electronics%20%26%20Control-green.svg)
![Numerical Solver](https://img.shields.io/badge/Solver-ode45-orange.svg)

## 📌 Project Overview
This repository contains a comprehensive first-principles simulation of a non-linear, non-minimum phase **Buck-Boost DC-DC Converter** implemented in MATLAB. The system dynamics are formulated using state-space average differential equations and solved numerically using high-precision Runge-Kutta numerical integration (`ode45`).

The project systematically benchmarks three control strategies under severe **line disturbances** (input voltage sag: $12\text{ V} \to 10\text{ V}$) and **load disturbances** (load resistance step: $18\,\Omega \to 9\,\Omega$):
1. **Uncontrolled (Open-Loop)** converter with fixed duty cycle ($D = 0.6$).
2. **Single-Loop PI Controller** featuring soft-start ramping, steady-state feedforward, and conditional integration anti-windup.
3. **Two-Loop Cascade PI Controller** featuring an outer voltage regulation loop, current reference limiting ($I_{\max} = 10\text{ A}$), and a fast inner inductor current tracking loop.

---

## ⚡ Mathematical Model & State-Space Equations
The average non-linear dynamics of the continuous-conduction-mode (CCM) Buck-Boost converter are defined as:

$$\frac{di_L}{dt} = \frac{D \cdot V_{in} - (1 - D) \cdot V}{L}$$

$$\frac{dV}{dt} = \frac{(1 - D) \cdot i_L - \frac{V}{R}}{C}$$

Where:
* $i_L$: Inductor current
* $V$: Output capacitor voltage magnitude
* $D$: Converter duty cycle ($D \in [D_{\min}, D_{\max}]$)
* $V_{in}$: Input DC rail voltage
* $R$: Equivalent resistive load
* $L = 1\text{ mH}$, $C = 220\,\mu\text{F} \text{ or } 1000\,\mu\text{F}$

---

## 🏗️ Control Architectures

### 1. Single-Loop PI Controller with Feedforward & Anti-Windup
* **Reference Trajectory:** Soft-start command to eliminate startup inrush current:
  $$V_{ref\_cmd}(t) = V_{ref} \cdot \min\left(\frac{t}{T_{soft}}, 1\right)$$
* **Feedforward Term:** Computed from ideal buck-boost conversion ratio:
  $$D_{ff} = \frac{V_{ref\_cmd}}{V_{in} + V_{ref\_cmd}}$$
* **PI Law & Anti-Windup:** Conditional integration halts integrator accumulation during control saturation to prevent windup overshoot.

### 2. Two-Loop Cascade PI Architecture
* **Outer Voltage Loop:**
  $$I_{ref} = \text{sat}\left(K_{pv} e_v(t) + K_{iv} \int e_v(t) \, dt, \, 0, \, I_{\max}\right)$$
* **Inner Current Loop:**
  $$D = \text{sat}\left(D_{ff} + K_{pi} e_i(t) + K_{ii} \int e_i(t) \, dt, \, D_{\min}, \, D_{\max}\right)$$
* Inherent hardware protection through current bounding ($I_{\max} = 10\text{ A}$) to protect switching semiconductor devices.

---

## 📊 Performance Benchmarks

| Metric | Open-Loop ($D=0.6$) | Single-Loop PI ($C=220\,\mu\text{F}$) | Single-Loop PI ($C=1000\,\mu\text{F}$) | Cascade PI ($C=1000\,\mu\text{F}$) |
| :--- | :--- | :--- | :--- | :--- |
| **Target Voltage** | $18.00\text{ V}$ | $18.00\text{ V}$ | $18.00\text{ V}$ | $18.00\text{ V}$ |
| **Line Disturbance ($12\text{V} \to 10\text{V}$)** | Permanent drop to $15.00\text{ V}$ | Recovers to $18.00\text{ V}$ | Minimal dip ($<0.4\text{ V}$) | Rapid recovery via feedforward |
| **Load Disturbance ($18\,\Omega \to 9\,\Omega$)** | Plunges to $\approx 9.00\text{ V}$ | Transient undershoot | Significantly damped undershoot | Tight regulation with current tracking |
| **Steady-State Error** | High ($> 9\text{ V}$) | $0.00\text{ V}$ | $0.00\text{ V}$ | $0.00\text{ V}$ |
| **Peak Current Protection** | None | None | None | Clamped to $10.00\text{ A}$ |

---

## 📂 Repository Structure
```text
.
├── basic_model.m              # Open-loop benchmark model
├── buck_booster_PI.m          # Single-loop PI control (C = 220 uF)
├── PI_increse_capacitance.m   # Single-loop PI control with enhanced filter (C = 1000 uF)
├── cascade_pi.m               # Two-loop Cascade PI controller (Voltage + Current)
└── README.md                  # Project documentation
