package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import org.littletonrobotics.junction.Logger;

/**
 * Physics-based simulation of the robot's pneumatic system (Viair 90C + 2× Clippard AVT-PP-35).
 *
 * <p>Five-stock model connected by four flows:
 *
 * <pre>
 *  Atmosphere ──[Compressor]──▶ Storage ──[PRU14]──▶ Working ──[SY3240 supply]──▶ Cylinder (active side)
 *                                                                                  Cylinder (passive side) ──[Flow control → SY3240 return]──▶ Atmosphere
 * </pre>
 *
 * <p>Flows are computed via the ISA S75.01 compressible-gas Cv formula (subcritical and choked
 * regimes). Cylinder pressure and piston position are integrated with Euler steps.
 *
 * <p>Component Cv values:
 *
 * <ul>
 *   <li>PRU14 regulator: Cv = 0.149, derived from 5.3 scfm @ 90 psig inlet / 14.5 psi ΔP
 *   <li>SMC SY3240-5MZ solenoid (supply passage): Cv = 0.30
 *   <li>Adjustable flow-control valve: Cv = 0.19
 *   <li>Solenoid return passage: Cv = 0.30 (exhaust air re-enters the spool before reaching
 *       atmosphere)
 *   <li>Series exhaust (flow-control + solenoid return): Cv = 1/√(1/0.19² + 1/0.30²) ≈ 0.160
 * </ul>
 */
public class PneumaticsSimulator {
  // ── Viair 90C performance — linear regressions over datasheet points ──────
  //
  // Flow (scfm) vs gauge pressure (psig):
  //   data: 0→0.88, 10→0.50, 20→0.43, 30→0.36, 40→0.30, 50→0.27,
  //         60→0.25, 70→0.24, 80→0.24, 90→0.23, 100→0.22, 110→0.22
  //   fit:  Q = 0.5754 − 0.004189 × p
  private static final double FLOW_SLOPE = -0.004189;
  private static final double FLOW_INTERCEPT = 0.5754;

  // Current (A) vs gauge pressure (psig):
  //   data: 0→7.0, 10→8.0, 20→8.0, 30→9.0, 40→9.0, 50→9.0,
  //         60→10.0, 70→10.0, 80→10.0, 90→11.0, 100→11.0, 110→10.0
  //   fit:  I = 7.603 + 0.03147 × p
  private static final double AMP_SLOPE = 0.03147;
  private static final double AMP_INTERCEPT = 7.603;

  // ── Thermodynamic / unit constants ────────────────────────────────────────
  /** Standard atmospheric pressure (psia). */
  private static final double P_ATM_PSIA = 14.696;

  /** Standard air temperature (°R = 68 °F). */
  private static final double T_AIR_R = 528.0;

  /** Converts scfm to in³/s at standard conditions (1728 in³/ft³ ÷ 60 s/min). */
  private static final double IN3_PER_S_PER_SCFM = 28.8;

  // ── Storage tank ──────────────────────────────────────────────────────────
  /** 2 × Clippard AVT-PP-35 (in³). */
  private static final double V_TANK_IN3 = 70.0;

  /** Compressor cut-off pressure (psig). */
  private static final double P_CUTOFF_PSIG = 120.0;

  // ── Pressure-reducing regulator: Nitra PRU14 ─────────────────────────────
  /**
   * Cv derived from datasheet: 5.3 scfm at P1=90 psig, ΔP=14.5 psi → P2=75.5 psig=90.196 psia.
   * P2/P1=0.861 (subcritical). Cv = Q / (22.67 × √(ΔP × P2 / T)) = 5.3 / (22.67 × √(2.477)) =
   * 0.149.
   */
  private static final double CV_REGULATOR = 0.149;

  /** Regulator outlet set point (psig). */
  private static final double P_REG_PSIG = 60.0;

  // ── Working volume (manifold + tubing between regulator outlet and solenoid inlet) ──
  private static final double V_WORKING_IN3 = 5.0;

  // ── Solenoid valve: SMC SY3240-5MZ (supply passage) ─────────────────────
  private static final double CV_SOLENOID = 0.30;

  // ── Exhaust path: flow-control valve (Cv 0.19) in series with solenoid return passage (Cv 0.30)
  // Series Cv = 1 / √(1/Cv1² + 1/Cv2²)
  private static final double CV_EXHAUST =
      1.0 / Math.sqrt(1.0 / (0.19 * 0.19) + 1.0 / (0.30 * 0.30));

  // ── Cylinder geometry: 2 × (3/4" bore, 7" stroke, 1/4" rod) ─────────────
  /** Total bore area across both cylinders (in²). */
  private static final double A_BORE_IN2 = 2.0 * Math.PI * Math.pow(0.375, 2);

  /** Total annular (rod-side) area across both cylinders (in²). */
  private static final double A_ROD_IN2 = 2.0 * Math.PI * (Math.pow(0.375, 2) - Math.pow(0.125, 2));

  private static final double STROKE_IN = 7.0;

  /** Dead volume at each end (prevents V=0 singularity in pressure ODE). */
  private static final double V_DEAD_BORE_IN3 = A_BORE_IN2 * 0.5;

  private static final double V_DEAD_ROD_IN3 = A_ROD_IN2 * 0.5;

  // ── Piston dynamics: overdamped first-order (v = F / b) ──────────────────
  /** Viscous damping constant (lbf·s/in). Determines actuation speed. */
  private static final double PISTON_DAMPING = 2.0;

  // ── State variables ───────────────────────────────────────────────────────
  /** Storage tank pressure (psia). */
  private double pStorage;

  /** Working-volume pressure between regulator and solenoid (psia). */
  private double pWorking;

  /** Bore-side cylinder pressure (psia). */
  private double pBore;

  /** Rod-side cylinder pressure (psia). */
  private double pRod;

  /** Piston position, normalised [0 = fully retracted, 1 = fully extended]. */
  private double xPiston;

  private boolean compressorRunning = false;
  private double compressorCurrentAmps = 0.0;

  private final DoubleSolenoidSim intakeArmSim;
  private final REVPHSim revphSim;

  public PneumaticsSimulator(DoubleSolenoidSim intakeArmSim, REVPHSim revphSim) {
    this.intakeArmSim = intakeArmSim;
    this.revphSim = revphSim;

    // Initial conditions: tank full, working at set point, cylinder fully retracted.
    // Rod side was last pressurized (solenoid started in kReverse); bore side at atmospheric.
    pStorage = P_CUTOFF_PSIG + P_ATM_PSIA;
    pWorking = P_REG_PSIG + P_ATM_PSIA;
    pBore = P_ATM_PSIA;
    pRod = P_REG_PSIG + P_ATM_PSIA;
    xPiston = 0.0;

    revphSim.setCompressorOn(false);
    revphSim.setPressureSwitch(true);
    revphSim.setCompressorCurrent(0.0);
  }

  public void update(double dtSeconds) {
    DoubleSolenoid.Value solenoidValue = intakeArmSim.get();

    // ── Current cylinder volumes ──────────────────────────────────────────
    double vBore = V_DEAD_BORE_IN3 + A_BORE_IN2 * STROKE_IN * xPiston;
    double vRod = V_DEAD_ROD_IN3 + A_ROD_IN2 * STROKE_IN * (1.0 - xPiston);

    // ── Piston velocity (overdamped: v = F_net / b) ───────────────────────
    // Positive = extending (bore side expanding, rod side compressing).
    double fNet = (pBore - P_ATM_PSIA) * A_BORE_IN2 - (pRod - P_ATM_PSIA) * A_ROD_IN2;
    double vPiston = fNet / PISTON_DAMPING; // in/s

    // Clamp velocity at hard end stops — a piston already at its mechanical
    // limit cannot move further into the stop, so dV/dt must be zero there.
    // Without this, phantom volume changes trigger spurious pressure flows.
    if (xPiston <= 0.0 && vPiston < 0) vPiston = 0;
    if (xPiston >= 1.0 && vPiston > 0) vPiston = 0;

    // dV/dt for each side (in³/s)
    double dvBoreDt = A_BORE_IN2 * vPiston;
    double dvRodDt = -A_ROD_IN2 * vPiston;

    // ── Solenoid-directed flows (scfm, always ≥ 0) ────────────────────────
    // Supply: working volume → active cylinder side, through solenoid supply passage (Cv 0.30).
    // Exhaust: passive cylinder side → atmosphere, through flow-control valve then solenoid
    //          return passage in series (combined Cv ≈ 0.160).
    double qBoreNetScfm = 0.0; // positive = net inflow to bore side
    double qRodNetScfm = 0.0; // positive = net inflow to rod side
    double qSupplyScfm = 0.0; // consumed from working volume

    if (solenoidValue == DoubleSolenoid.Value.kForward) {
      // Extend: supply → bore, exhaust ← rod
      qSupplyScfm = flowScfm(CV_SOLENOID, pWorking, pBore);
      double qExhaustScfm = flowScfm(CV_EXHAUST, pRod, P_ATM_PSIA);
      qBoreNetScfm = +qSupplyScfm;
      qRodNetScfm = -qExhaustScfm;
    } else if (solenoidValue == DoubleSolenoid.Value.kReverse) {
      // Retract: supply → rod, exhaust ← bore
      qSupplyScfm = flowScfm(CV_SOLENOID, pWorking, pRod);
      double qExhaustScfm = flowScfm(CV_EXHAUST, pBore, P_ATM_PSIA);
      qRodNetScfm = +qSupplyScfm;
      qBoreNetScfm = -qExhaustScfm;
    }
    // kOff: both passages blocked, all flows remain zero.

    // ── Regulator flow: storage → working (PRU14, Cv 0.149) ──────────────
    // The regulator is closed when pWorking is at or above the set point.
    double pRegSetPsia = P_REG_PSIG + P_ATM_PSIA;
    double qRegScfm = 0.0;
    if (pStorage > pWorking && pWorking < pRegSetPsia) {
      qRegScfm = flowScfm(CV_REGULATOR, pStorage, pWorking);
    }

    // ── Compressor flow: atmosphere → storage (Viair 90C) ─────────────────
    double pStorageGauge = pStorage - P_ATM_PSIA;
    compressorRunning = DriverStation.isEnabled() && pStorageGauge < P_CUTOFF_PSIG;
    compressorCurrentAmps = compressorRunning ? AMP_INTERCEPT + AMP_SLOPE * pStorageGauge : 0.0;
    double qCompressorScfm = compressorRunning ? FLOW_INTERCEPT + FLOW_SLOPE * pStorageGauge : 0.0;

    // ── Pressure ODEs (isothermal ideal gas) ──────────────────────────────
    // For a variable-volume chamber: dp/dt = (P_atm × Q_net × K − p × dV/dt) / V
    // where K = IN3_PER_S_PER_SCFM converts scfm to in³/s at standard conditions.
    double k = P_ATM_PSIA * IN3_PER_S_PER_SCFM;

    double dpStorageDt = k * (qCompressorScfm - qRegScfm) / V_TANK_IN3;
    double dpWorkingDt = k * (qRegScfm - qSupplyScfm) / V_WORKING_IN3;
    double dpBoreDt = (k * qBoreNetScfm - pBore * dvBoreDt) / vBore;
    double dpRodDt = (k * qRodNetScfm - pRod * dvRodDt) / vRod;

    // ── Euler integration ─────────────────────────────────────────────────
    pStorage = Math.max(P_ATM_PSIA, pStorage + dpStorageDt * dtSeconds);
    // Regulator hard-caps working pressure at set point
    pWorking = Math.max(P_ATM_PSIA, Math.min(pRegSetPsia, pWorking + dpWorkingDt * dtSeconds));
    pBore = Math.max(P_ATM_PSIA, pBore + dpBoreDt * dtSeconds);
    pRod = Math.max(P_ATM_PSIA, pRod + dpRodDt * dtSeconds);

    // Piston position (normalised): clamp at hard end stops
    xPiston = Math.min(1.0, Math.max(0.0, xPiston + (vPiston / STROKE_IN) * dtSeconds));

    // ── Sync HAL ──────────────────────────────────────────────────────────
    revphSim.setCompressorOn(compressorRunning);
    revphSim.setPressureSwitch(pStorageGauge >= P_CUTOFF_PSIG);
    revphSim.setCompressorCurrent(compressorCurrentAmps);

    // ── Telemetry ─────────────────────────────────────────────────────────
    Logger.recordOutput("Pneumatics/TankPressurePsi", pStorageGauge);
    Logger.recordOutput("Pneumatics/WorkingPressurePsi", pWorking - P_ATM_PSIA);
    Logger.recordOutput("Pneumatics/BorePressurePsi", pBore - P_ATM_PSIA);
    Logger.recordOutput("Pneumatics/RodPressurePsi", pRod - P_ATM_PSIA);
    Logger.recordOutput("Pneumatics/PistonPosition", xPiston);
    Logger.recordOutput("Pneumatics/CompressorRunning", compressorRunning);
    Logger.recordOutput("Pneumatics/CompressorCurrentAmps", compressorCurrentAmps);
  }

  /**
   * Compressible gas flow through a valve (ISA S75.01 simplified, air at 68 °F).
   *
   * <ul>
   *   <li>Subcritical (P2/P1 &gt; 0.528): Q = 22.67 × Cv × √(ΔP × P2 / T)
   *   <li>Choked (P2/P1 ≤ 0.528): Q = 22.67 × Cv × P1 × 0.471 / √T
   * </ul>
   *
   * @param cv valve flow coefficient
   * @param p1 upstream absolute pressure (psia)
   * @param p2 downstream absolute pressure (psia)
   * @return flow rate in scfm (≥ 0; zero when p1 ≤ p2)
   */
  private static double flowScfm(double cv, double p1, double p2) {
    if (p1 <= p2) return 0.0;
    double sqrtT = Math.sqrt(T_AIR_R);
    if (p2 / p1 > 0.528) {
      return 22.67 * cv * Math.sqrt((p1 - p2) * p2 / T_AIR_R);
    } else {
      return 22.67 * cv * p1 * 0.471 / sqrtT;
    }
  }

  public double getCompressorCurrentAmps() {
    return compressorCurrentAmps;
  }

  public double getTankPressurePsi() {
    return pStorage - P_ATM_PSIA;
  }

  public boolean isCompressorRunning() {
    return compressorRunning;
  }
}
