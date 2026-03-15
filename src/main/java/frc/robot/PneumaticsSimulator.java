package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import org.littletonrobotics.junction.Logger;

/** Physics-based simulation of the robot's pneumatic system (Viair 90C + 2× Clippard AVT-PP-35). */
public class PneumaticsSimulator {
  // Viair 90C performance data (gauge pressure psi → flow cfm / current amps)
  private static final InterpolatingDoubleTreeMap FLOW_CFM_MAP = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap AMP_DRAW_MAP = new InterpolatingDoubleTreeMap();

  static {
    FLOW_CFM_MAP.put(0.0, 0.88);
    FLOW_CFM_MAP.put(10.0, 0.50);
    FLOW_CFM_MAP.put(20.0, 0.43);
    FLOW_CFM_MAP.put(30.0, 0.36);
    FLOW_CFM_MAP.put(40.0, 0.30);
    FLOW_CFM_MAP.put(50.0, 0.27);
    FLOW_CFM_MAP.put(60.0, 0.25);
    FLOW_CFM_MAP.put(70.0, 0.24);
    FLOW_CFM_MAP.put(80.0, 0.24);
    FLOW_CFM_MAP.put(90.0, 0.23);
    FLOW_CFM_MAP.put(100.0, 0.22);
    FLOW_CFM_MAP.put(110.0, 0.22);

    AMP_DRAW_MAP.put(0.0, 7.0);
    AMP_DRAW_MAP.put(10.0, 8.0);
    AMP_DRAW_MAP.put(20.0, 8.0);
    AMP_DRAW_MAP.put(30.0, 9.0);
    AMP_DRAW_MAP.put(40.0, 9.0);
    AMP_DRAW_MAP.put(50.0, 9.0);
    AMP_DRAW_MAP.put(60.0, 10.0);
    AMP_DRAW_MAP.put(70.0, 10.0);
    AMP_DRAW_MAP.put(80.0, 10.0);
    AMP_DRAW_MAP.put(90.0, 11.0);
    AMP_DRAW_MAP.put(100.0, 11.0);
    AMP_DRAW_MAP.put(110.0, 10.0);
  }

  // System geometry
  private static final double V_TANK_IN3 = 70.0; // 2 × 35 in³ Clippard AVT-PP-35
  private static final double P_ATM_PSI = 14.696;
  private static final double P_CUTOFF_PSI = 120.0; // compressor cut-off pressure

  // Intake arm cylinders: 2 cylinders, 3/4" bore, 7" stroke, 1/4" rod
  private static final double V_EXTEND_IN3 = 2.0 * Math.PI * Math.pow(0.375, 2) * 7.0;
  private static final double V_RETRACT_IN3 =
      2.0 * Math.PI * (Math.pow(0.375, 2) - Math.pow(0.125, 2)) * 7.0;

  // Working pressure (absolute) used to compute air consumed per actuation
  private static final double P_WORKING_ABS_PSI = 60.0 + P_ATM_PSI;

  private double pressurePsi = P_CUTOFF_PSI;
  private boolean compressorRunning = false;
  private double compressorCurrentAmps = 0.0;
  private DoubleSolenoid.Value lastSolenoidValue = DoubleSolenoid.Value.kReverse;

  private final DoubleSolenoidSim intakeArmSim;
  private final REVPHSim revphSim;

  public PneumaticsSimulator(DoubleSolenoidSim intakeArmSim, REVPHSim revphSim) {
    this.intakeArmSim = intakeArmSim;
    this.revphSim = revphSim;
    // Tank starts full — compressor is off and pressure switch is engaged
    revphSim.setCompressorOn(false);
    revphSim.setPressureSwitch(true);
    revphSim.setCompressorCurrent(0.0);
  }

  public void update(double dtSeconds) {
    // Detect solenoid transitions and consume air
    DoubleSolenoid.Value solenoidValue = intakeArmSim.get();
    if (solenoidValue != lastSolenoidValue) {
      if (solenoidValue == DoubleSolenoid.Value.kForward) {
        pressurePsi -= P_WORKING_ABS_PSI * V_EXTEND_IN3 / V_TANK_IN3;
      } else if (solenoidValue == DoubleSolenoid.Value.kReverse) {
        pressurePsi -= P_WORKING_ABS_PSI * V_RETRACT_IN3 / V_TANK_IN3;
      }
      pressurePsi = Math.max(0.0, pressurePsi);
      lastSolenoidValue = solenoidValue;
    }

    // Sample amp draw at the pressure the compressor is working against this step,
    // then advance pressure. This keeps current and dP/dt consistent with the same
    // operating point.
    compressorRunning = pressurePsi < P_CUTOFF_PSI;
    compressorCurrentAmps = compressorRunning ? AMP_DRAW_MAP.get(pressurePsi) : 0.0;
    if (compressorRunning) {
      double dPdtPsiPerSec = FLOW_CFM_MAP.get(pressurePsi) * 28.8 * P_ATM_PSI / V_TANK_IN3;
      pressurePsi = Math.min(pressurePsi + dPdtPsiPerSec * dtSeconds, P_CUTOFF_PSI);
    }

    // Sync HAL state so Compressor.isEnabled() / getPressureSwitchValue() / PDH current are
    // accurate
    revphSim.setCompressorOn(compressorRunning);
    revphSim.setPressureSwitch(pressurePsi >= P_CUTOFF_PSI);
    revphSim.setCompressorCurrent(compressorCurrentAmps);

    Logger.recordOutput("Pneumatics/TankPressurePsi", pressurePsi);
    Logger.recordOutput("Pneumatics/CompressorRunning", compressorRunning);
    Logger.recordOutput("Pneumatics/CompressorCurrentAmps", compressorCurrentAmps);
  }

  public double getCompressorCurrentAmps() {
    return compressorCurrentAmps;
  }

  public double getTankPressurePsi() {
    return pressurePsi;
  }

  public boolean isCompressorRunning() {
    return compressorRunning;
  }
}
