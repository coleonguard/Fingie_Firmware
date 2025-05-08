Thought for 5 seconds


# Unified Prosthetic-Hand Controller

### *Subsystem project specification*

**Reach → Grasp → Lift → Lower → Release Controller - SW-only Verification v1.0 (2025-05-08)**

---

## 0. Notation & Acronyms

| Symbol / term  | Meaning                                                 |
| -------------- | ------------------------------------------------------- |
| $d_f$          | Kalman-filtered VL6180X range for finger *f* (mm)       |
| $d_\text{app}$ | **40 mm** – approach-phase entry threshold              |
| $d_\text{con}$ | **5 mm** – contact threshold (VL6180X unreliable below) |
| $θ_f$          | commanded finger position (degrees)                     |
| $τ_f$          | commanded motor current ≈ torque (A)                    |
| $T_c$          | hard current clamp **0.6 A** (≈2× heaviest load)        |
| $a_z$          | world-frame specific force along gravity (+ up, m s⁻²)  |
| $\|ω\|$        | angular-rate magnitude from IMU (° s⁻¹)                 |
| KF             | Kalman Filter (per range sensor)                        |
| FSM            | finite-state machine                                    |
| AHSC           | *Ability-Hand Serial Client* mock                       |
| ND-JSON        | newline-delimited JSON log (one dict per 20 Hz cycle)   |

---

## 1. Goals & External Contracts

### 1.1 Functional Goals

| ID      | Description                                                                                                                                                                                           |       |            |       |                |      |                                                                                          |
| ------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----- | ---------- | ----- | -------------- | ---- | ---------------------------------------------------------------------------------------- |
| **G-1** | Close each finger proportionally once its MCP range $d_f<d_\text{app}$; switch the finger to torque mode when $d_f≤d_\text{con}$ **or** the derivative of motor current $\dot I_f$ crosses threshold. |       |            |       |                |      |                                                                                          |
| **G-2** | Detect table contact and govern finger **release using IMU alone** (downward phase → impact impulse → 0.3 s stationary).                                                                              |       |            |       |                |      |                                                                                          |
| **G-3** | Global safety: (                                                                                                                                                                                      | Δθ\_f | ≤8°/cycle, | Δτ\_f | ≤0.05 A/cycle, | τ\_f | ≤T\_c). Any loop over-run > 100 ms drives all torques to 0 and logs `fault:"COMM_LOSS"`. |
| **G-4** | Produce schema-valid ND-JSON log at 20 Hz containing *all* raw/filtered sensor values, commands, states and faults.                                                                                   |       |            |       |                |      |                                                                                          |
| **G-5** | Pass an exhaustive **software-only** test-suite (see §5) so integration with real hardware can proceed with confidence.                                                                               |       |            |       |                |      |                                                                                          |

### 1.2 Runtime Interfaces (mocked in tests)

| Dir | Name              | API / Mock                  | Data                                              |
| --- | ----------------- | --------------------------- | ------------------------------------------------- |
| ↑   | Proximity sensors | `MockVL6180X` via `MockMux` | scripted mm sequence                              |
| ↑   | IMU               | `MockIMU` (CSV trace)       | `(ax,ay,az,gx,gy,gz)` @200 Hz                     |
| ↑   | Motors            | `MockHand`                  | position & current with first-order lag (τ=40 ms) |
| ↓   | Log file          | ND-JSON                     | one dict / control cycle                          |

---

## 2. Detailed Architecture & Sub-blocks

### 2.1 Sequence Diagram (software mocks)

```
MockIMU 200 Hz ──► IMUInterface ─┐
MockMux+VL ──► ProximityManager ─┤
                                  ▼
               ┌────────── UnifiedController (20 Hz loop) ──────────┐
               │ 1 read caches                                      │
               │ 2 per-finger FSM (APPROACH/PROP/CONTACT)           │
               │ 3 hand  FSM  (REACH/LIFT/LOWER/RELEASE/RETRACT)    │
               │ 4 safety caps (rate, clamp)                        │
               │ 5 send cmd to MockHand                             │
               │ 6 append ND-JSON log                               │
               └────────────────────────────────────────────────────┘
```

### 2.2 Sub-block Specification

| ID     | Name                   | Core Algo                               | Mock dependencies        | Risk             |
| ------ | ---------------------- | --------------------------------------- | ------------------------ | ---------------- |
| **P**  | `ProximityManager`     | VL6180X init, retry, KF, 0-mm mask      | `MockVL6180X`, `MockMux` | glitch masking   |
| **I**  | `IMUInterface`         | DMA-style buffer → Madgwick β=0.04      | `MockIMU`                | orientation sign |
| **A**  | `AbilityHandInterface` | Position / torque TxRx, watchdog 100 ms | `MockHand`               | timeout handling |
| **F1** | Finger FSM             | thresholds, velocity limiter 8°         | P, A                     | monotonicity     |
| **F2** | Hand FSM               | a\_z impulse + settle + release         | I, A                     | false release    |
| **L**  | Logger                 | ND-JSON schema validator                | —                        | schema drift     |

---

## 3. Algorithms & Theory

### 3.1 Distance → Position Mapping

$$
θ_f = \begin{cases}
0 & d_f ≥ d_\text{app}\\[4pt]
θ_{\max}\dfrac{d_\text{app}-d_f}{d_\text{app}-d_\text{con}} & d_\text{con}<d_f<d_\text{app}\\[8pt]
θ_{\max}=100° & d_f ≤ d_\text{con}
\end{cases}
$$

Rate-limited: $|Δθ_f|\le 8°$ per 50 ms loop.

### 3.2 Contact-phase Torque Regulation

*Initial* $τ_f=0.30 A$.
Every cycle:

```
if I_f > 0.45:   τ_f ← 0.9 τ_f      # relieve
if slip:         τ_f ← min(τ_f+0.05, 0.6)
```

Slip = $ \dot I_f < -0.05$ A s⁻¹ for ≥2 cycles.

### 3.3 IMU-only Table-Contact Detection

| Condition           | Numeric threshold                       |        |                         |
| ------------------- | --------------------------------------- | ------ | ----------------------- |
| **Lowering starts** | $a_z < g-4\;m s^{-2}$ for ≥50 ms        |        |                         |
| **Impact impulse**  | high-pass $a_z > 7.8\;m s^{-2}$         |        |                         |
| **Stationary**      | 0.3 s window: (                         | a\_z-g | <1), $\|ω\|<10° s^{-1}$ |
| **Release**         | Stationary **and** all currents < 0.2 A |        |                         |

### 3.4 Kalman Glitch Mask

If measurement ∈ {0, 255} mm → treat as missing, propagate previous estimate.

---

## 4. Software-Only Acceptance Criteria & Test Cases

| ID       | Scope  | Title                  | Fixture / Method               | Expected Result           |
| -------- | ------ | ---------------------- | ------------------------------ | ------------------------- |
| **U-01** | Unit   | Kalman glitch mask     | feed `[30,0,0,28]` mm          | output never 0            |
| **U-02** | Unit   | Velocity limiter       | cmd 0→100° in 1 loop           | first call ≤ 8°           |
| **U-03** | Unit   | Torque clamp           | request 1 A                    | `τ_f≤0.6 A`               |
| **U-04** | Unit   | IMU sign self-test     | startup `az≈−9.8`              | raises `OrientationError` |
| **C-01** | Comp   | Finger θ monotonic     | `d=[45,35,25,15,5]`            | θ strictly ↑              |
| **C-02** | Comp   | dI/dt contact trigger  | distance stuck 7 mm, `I` spike | FSM enters CONTACT        |
| **C-03** | Comp   | Hand FSM release       | CSV: down→impact→settle        | states sequence OK        |
| **C-04** | Comp   | Watchdog trip          | stall MockHand 120 ms          | torques 0, fault flag     |
| **I-01** | Int    | 20 Hz timing           | FakeClock 1000 loops           | each ≤ 15 ms              |
| **I-02** | Int    | Log schema             | run 200 loops → jsonschema     | 100 % valid               |
| **I-03** | Int    | Deterministic replay   | run twice seed 42              | logs SHA-256 equal        |
| **I-04** | Int    | Thread safety          | run with `-Xfaulthandler`      | no race warnings          |
| **S-01** | System | Monte-Carlo 100 grasps | random latency/noise sim       | ≥95 success, 0 deadlock   |

---

## 5. Test Harness & CI

* `tests/mocks.py` – all mock classes + `FakeClock`.
* `pytest` + `pytest-cov`; coverage gate 90 %.
* `jsonschema` validates log entries.
* GitHub Actions matrix (`py3.10`, `py3.11`) runs full suite in < 30 s.

---

## 6. Completion Definition

Project is **ready for first hardware hookup** when:

1. All tests **U-**, **C-**, **I-**, **S-01** pass in CI.
2. Line-coverage ≥ 90 % across core modules.
3. `flake8`/`ruff` emit zero warnings.
4. Example log `example_log.ndjson` validates against schema and is reproducible under seed 42.

This document, plus the CI-green codebase, removes all ambiguity; any failure on real hardware will be electrical or mechanical, **not logic**.
