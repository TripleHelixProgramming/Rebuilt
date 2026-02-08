package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants.MotorConstants.NEOVortexConstants;

public final class FeederConstants {
  public static final class SpindexerConstants {
    // Geometry
    public static final Distance radius = Inches.of(3.0);

    // Motor
    public static final double motorReduction = 1.0;
    public static final LinearVelocity maxTangentialVelocity =
        MetersPerSecond.of(
            NEOVortexConstants.kFreeSpeed.in(RadiansPerSecond)
                * radius.in(Meters)
                / motorReduction);

    // Encoder
    public static final double encoderPositionFactor = 2.0 * Math.PI / motorReduction; // Meters
    public static final double encoderVelocityFactor = encoderPositionFactor / 60.0; // Meters/sec

    // Simulation
    public static final double kPSim = 0.05;
    public static final DCMotor gearbox = DCMotor.getNeoVortex(1);
  }

  public static final class KickerConstants {
    // Geometry
    public static final Distance radius = Inches.of(1.5);

    // Motor
    public static final double motorReduction = 1.0;
    public static final LinearVelocity maxTangentialVelocity =
        MetersPerSecond.of(
            NEOVortexConstants.kFreeSpeed.in(RadiansPerSecond)
                * radius.in(Meters)
                / motorReduction);

    // Encoder
    public static final double encoderPositionFactor = 2.0 * Math.PI / motorReduction; // Meters
    public static final double encoderVelocityFactor = encoderPositionFactor / 60.0; // Meters/sec

    // Simulation
    public static final double kPSim = 0.05;
    public static final DCMotor gearbox = DCMotor.getNeoVortex(1);
  }
}
