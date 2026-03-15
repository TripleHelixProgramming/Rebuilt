# Pneumatics Simulation — How It Works

This document explains the physics model behind `PneumaticsSimulator.java`.

---

## Why simulate pneumatics at all?

The robot arm is driven by two pneumatic cylinders — small pistons pushed by compressed air. Without
a pneumatic model, the arm teleports instantly between positions and the compressor never runs, making
it impossible to test anything that depends on air pressure (does the tank run dry mid-match? does
the arm extend before the intake command times out?).

The goal of this simulator is to make the virtual robot behave like the real one: air flows through
real components with real resistances, pressure builds up and bleeds down at physically correct
rates, and the piston moves only as fast as the air can get in and out.

---

## The five-stock model

The pneumatic system is modelled as five **stocks** — volumes of air each characterised by a single
pressure — connected by **flows**.

```
Atmosphere ──[Compressor]──▶ Storage ──[PRU14 Regulator]──▶ Working ──[SY3240 Solenoid]──▶ Cylinder (active side)
                                                                                             Cylinder (passive side) ──[Flow control → Solenoid return]──▶ Atmosphere
```

| Stock | Physical thing | Volume |
|---|---|---|
| Atmosphere | The room | Infinite (fixed at 14.696 psia) |
| Storage | Two Clippard AVT-PP-35 tanks | 70 in³ |
| Working | Manifold and tubing between regulator and solenoid | ~5 in³ (estimated) |
| Cylinder (active side) | Whichever side of the piston is being pressurized | Changes as piston moves |
| Cylinder (passive side) | The other side, exhausting to atmosphere | Changes as piston moves |

The **active** and **passive** sides swap every time the solenoid reverses direction. When extending,
the bore (large) side is active; when retracting, the rod (annular) side is active.

---

## The ideal gas law

The pressure dynamics are derived from the ideal gas law:

$$PV = nRT$$

Assuming constant temperature (the **isothermal** approximation — reasonable for the short
timescales of a match), $P \propto 1/V$ for a fixed amount of gas.

For a chamber where both pressure and volume change over time, differentiating both sides with
respect to time gives:

$$\frac{d(PV)}{dt} = \frac{d(nRT)}{dt} = RT \frac{dn}{dt}$$

The right-hand side is the rate at which moles of air are flowing in or out, which is proportional
to the volumetric flow rate $Q$ measured at standard conditions. Rearranging:

$$V \frac{dP}{dt} + P \frac{dV}{dt} = P_\text{atm} \cdot Q_\text{net}$$

Solving for the pressure derivative:

$$\boxed{\frac{dP}{dt} = \frac{P_\text{atm} \cdot Q_\text{net} - P \cdot \dot{V}}{V}}$$

The two terms in the numerator have distinct physical meanings:
- $P_\text{atm} \cdot Q_\text{net}$: net mass inflow raises pressure; net outflow (negative
  $Q_\text{net}$) lowers it.
- $P \cdot \dot{V}$: an **expanding** chamber ($\dot{V} > 0$, e.g. piston moving outward) drops
  pressure even with no mass flow, because the same air now occupies a larger volume.

This equation is applied independently to each stock on every simulation timestep.

> **Unit note.** $Q$ in the code is in standard cubic feet per minute (scfm). One scfm equals
> 28.8 in³/s. $P$ is in psia (pounds per square inch absolute). The factor of 28.8 keeps the units
> consistent: $P_\text{atm}$ [psia] × $Q$ [in³/s] gives psia·in³/s, and dividing by $V$ [in³]
> gives psia/s.

---

## How flow rates are computed — the Cv formula

Every connection between stocks is a restriction — a valve, regulator, or tubing. The resistance
of a flow component is characterised by its **flow coefficient $C_v$**, defined such that:

> A valve with $C_v = 1$ passes **1 US gallon per minute** of water at a pressure drop of **1 psi**.

For compressible gas (air) the formula has two regimes depending on whether the flow reaches the
speed of sound at the throat (choked flow).

**Subcritical** (pressure ratio $P_2/P_1 > 0.528$ — flow is subsonic):

$$Q_\text{scfm} = 22.67 \cdot C_v \cdot \sqrt{\frac{(P_1 - P_2) \cdot P_2}{T}}$$

**Choked** (pressure ratio $P_2/P_1 \leq 0.528$ — flow is sonic at the throat and cannot increase further):

$$Q_\text{scfm} = 22.67 \cdot C_v \cdot P_1 \cdot \frac{0.471}{\sqrt{T}}$$

where $P_1$, $P_2$ are absolute pressures in psia and $T = 528\ ^\circ\text{R}$ (68 °F).

The critical pressure ratio of 0.528 comes from isentropic nozzle theory for air
($\gamma = 1.4$): $\left(\frac{2}{\gamma+1}\right)^{\gamma/(\gamma-1)} = 0.528$. Once the
downstream pressure is less than 52.8 % of the upstream pressure, the throat is choked and
increasing the pressure ratio further does not increase the mass flow.

### Component Cv values

| Component | $C_v$ | How obtained |
|---|---|---|
| PRU14 pressure-reducing regulator | 0.149 | Calculated from datasheet: 5.3 scfm at 90 psig inlet, 14.5 psi ΔP |
| SMC SY3240-5MZ solenoid (supply passage) | 0.30 | From SMC datasheet |
| Adjustable flow-control valve | 0.19 | Set to match 6.1 scfm at 100 psi |
| Solenoid return passage | 0.30 | Same spool, different flow path |

### Deriving the PRU14 Cv

The datasheet gives a single operating point: 5.3 scfm at $P_1 = 90\ \text{psig}$, $\Delta P = 14.5\ \text{psi}$.
That means $P_2 = 75.5\ \text{psig}$.

Converting to absolute: $P_1 = 104.7\ \text{psia}$, $P_2 = 90.2\ \text{psia}$.

Check for choked flow: $P_2/P_1 = 90.2/104.7 = 0.861 > 0.528$ → subcritical, use the first formula.

Solving for $C_v$:

$$C_v = \frac{Q_\text{scfm}}{22.67 \cdot \sqrt{(P_1 - P_2) \cdot P_2 / T}}
      = \frac{5.3}{22.67 \cdot \sqrt{14.5 \times 90.2 / 528}}
      = \frac{5.3}{22.67 \times 1.574}
      \approx 0.149$$

### Valves in series — the exhaust path

When the cylinder exhausts, air must travel through **two restrictions in series**: the
flow-control valve ($C_v = 0.19$) and then the solenoid return passage ($C_v = 0.30$) before
reaching atmosphere. Two restrictions in series behave like one restriction with a combined $C_v$:

$$\frac{1}{C_{v,\text{combined}}^2} = \frac{1}{C_{v,1}^2} + \frac{1}{C_{v,2}^2}$$

$$C_{v,\text{exhaust}} = \frac{1}{\sqrt{\dfrac{1}{0.19^2} + \dfrac{1}{0.30^2}}}
= \frac{1}{\sqrt{27.7 + 11.1}} = \frac{1}{\sqrt{38.8}} \approx 0.160$$

This formula follows from the fact that the pressure drops across series restrictions add:
$\Delta P_\text{total} = \Delta P_1 + \Delta P_2$. Since $\Delta P \propto (Q/C_v)^2$, summing
the squared reciprocals gives the combined resistance.

---

## The compressor

The Viair 90C is modelled as a flow source whose output and current draw both depend on the storage
tank pressure it is working against. Flow rate and current are each approximated by a **linear
regression** fit to the manufacturer's datasheet, using the 12 published operating points from
0 to 110 psig:

$$Q_\text{scfm} = 0.5754 - 0.004189 \cdot P_\text{gauge}$$

$$I_\text{A} = 7.603 + 0.03147 \cdot P_\text{gauge}$$

The regression is a deliberate simplification; the actual curves are slightly concave (flow drops
off faster at low pressure and flattens at high pressure) but a straight line captures the
first-order trend without requiring a lookup table.

The compressor is **interlock with the FRC enable signal**: it only runs when
`DriverStation.isEnabled()` is true, matching the behaviour of the REV PH firmware. The tank
holds pressure indefinitely while the robot is disabled.

---

## The pressure-reducing regulator (PRU14)

A pressure-reducing regulator senses its outlet pressure and throttles its internal valve to
maintain a set point. In this simulation it is modelled with two rules:

1. **It only flows when the working pressure is below the set point** (60 psig). When the
   downstream side is already at pressure, the regulator closes.
2. **When it does flow**, the flow rate is computed with the $C_v$ formula using the actual storage
   and working pressures. This captures the finite flow capacity of the regulator — it can't
   replenish the working volume instantaneously.

After integration each timestep, the working pressure is hard-clamped to the set point so it never
overshoots due to numerical error.

---

## The cylinder and piston

The two cylinders each have a **3/4 inch bore**, a **1/4 inch rod**, and a **7 inch stroke**.

| Quantity | Formula | Value |
|---|---|---|
| Bore area (both cylinders) | $2 \cdot \pi \cdot (0.375\ \text{in})^2$ | 0.884 in² |
| Rod-side annular area | $2 \cdot \pi \cdot (0.375^2 - 0.125^2)\ \text{in}^2$ | 0.785 in² |
| Full-stroke bore volume | bore area × 7 in | 6.19 in³ |
| Full-stroke rod volume | rod area × 7 in | 5.50 in³ |

A small dead volume (0.5 in of stroke equivalent) is added to each side to prevent division by zero
when the piston is at an end stop.

### Piston dynamics

The net force on the piston (positive = extending direction) is:

$$F_\text{net} = (P_\text{bore} - P_\text{atm}) \cdot A_\text{bore} - (P_\text{rod} - P_\text{atm}) \cdot A_\text{rod}$$

Gauge pressures are used because atmospheric pressure acts on both sides and cancels.

Rather than solving the full $F = ma$ ODE (which would require knowing the arm's rotational
inertia and modelling stiction), the simulator uses an **overdamped** model in which piston
velocity is instantaneously proportional to net force, divided by a viscous damping constant $b$:

$$v_\text{piston} = \frac{F_\text{net}}{b} \qquad b = 2\ \text{lbf·s/in}$$

This is equivalent to assuming the piston mass is negligible compared to the damping — physically
reasonable for small cylinders where air cushioning and seal friction dominate inertia. It also
avoids numerical instability from the stiff mass-spring-damper ODE at 20 ms timesteps.

The piston position $x \in [0, 1]$ (0 = fully retracted, 1 = fully extended) is integrated as:

$$\dot{x} = v_\text{piston} / L_\text{stroke}$$

Velocity is clamped to zero at the end stops before computing $\dot{V}$: a piston that has reached
its mechanical limit cannot change either cylinder volume, preventing phantom pressure flows that
would otherwise drain the storage tank while the robot is at rest.

---

## The simulation loop

Each 20 ms timestep, the following steps execute in order:

1. **Read solenoid state** from the WPILib `DoubleSolenoidSim`.
2. **Compute cylinder volumes** from the current piston position.
3. **Compute piston velocity** from the pressure-force balance; clamp at end stops.
4. **Compute $\dot{V}$** for both cylinder sides.
5. **Compute all flow rates** ($Q$ in scfm) through each component using the $C_v$ formula.
6. **Compute $dP/dt$** for all four pressure states using the ideal-gas ODE.
7. **Euler integration**: $P_\text{new} = P_\text{old} + (dP/dt) \cdot \Delta t$.
8. **Update piston position** from velocity.
9. **Write results** to AdvantageKit telemetry and to the HAL simulation layer.

The Euler method is first-order accurate in time. At 20 ms steps and the flow rates involved,
this is accurate enough for the timescales of interest (cylinder strokes take ~0.3–0.5 s).

---

## What the telemetry keys mean

| Key | Units | What to look for |
|---|---|---|
| `Pneumatics/TankPressurePsi` | psig | Holds at 120 while disabled; drops ~12 psig per full stroke, recovers at ~0.75 psi/s while compressor runs |
| `Pneumatics/WorkingPressurePsi` | psig | Stays near 60; brief dip during active strokes as solenoid demand exceeds regulator flow |
| `Pneumatics/BorePressurePsi` | psig | Rises from 0 to ~60 when extending; see known limitations for oscillation at stroke start |
| `Pneumatics/RodPressurePsi` | psig | Falls smoothly from ~60 to 0 as the rod side exhausts during extension |
| `Pneumatics/PistonPosition` | 0–1 | 0 = retracted, 1 = extended; full stroke takes ~0.35 s extend / ~0.40 s retract |
| `Pneumatics/CompressorRunning` | bool | False while disabled and at full tank; true only when enabled and tank below 120 psig |
| `Pneumatics/CompressorCurrentAmps` | A | 7.6–11 A, rising with pressure per the linear regression fit |

---

## Limitations and known simplifications

- **Bore pressure oscillation at stroke start**: at the moment the solenoid opens, the bore dead
  volume (0.44 in³) is smaller than the amount of air that can flow through the solenoid in a
  single 20 ms Euler step at full differential pressure. This causes a one-to-two tick overshoot
  that can log bore pressures well above the supply pressure before settling. The effect is
  numerical, not physical, and disappears once the piston begins moving and the bore volume grows.
- **Euler integration**: first-order explicit integration is adequate for stroke-scale dynamics
  (0.3–0.5 s) but is stiff with respect to the small working and dead volumes. Working pressure
  also shows tick-to-tick oscillation during active strokes for the same reason.
- **Isothermal assumption**: real fast compressions are closer to adiabatic ($\gamma = 1.4$),
  which would predict slightly higher temperatures and pressures during rapid strokes. For a 7 in
  stroke at ~40 in/s the error is small.
- **Linear compressor model**: the regression lines fit the datasheet well in the mid-range but
  slightly underestimate flow at low tank pressure and overestimate current at high pressure.
- **Overdamped piston**: ignores arm inertia and stiction. Actuation speed is tunable via the
  `PISTON_DAMPING` constant if the simulation runs too fast or too slow compared to reality.
- **Single working volume**: the actual manifold and tubing geometry is approximated as one lumped
  5 in³ volume.
- **No leakage**: the real system loses a small amount of air through seal leakage over time. This
  is negligible over a 2 min 30 s match.
