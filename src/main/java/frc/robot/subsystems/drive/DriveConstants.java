// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANBusPorts.CANHD;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;

public class DriveConstants {

  public static final String zeroRotationKey = "ZeroRotation";

  // Robot physical dimensions
  public static final Distance wheelBase = Inches.of(22.5);
  public static final Distance trackWidth = Inches.of(19.5);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(wheelBase.div(2.0), trackWidth.div(2.0)),
        new Translation2d(wheelBase.div(2.0), trackWidth.div(-2.0)),
        new Translation2d(wheelBase.div(-2.0), trackWidth.div(2.0)),
        new Translation2d(wheelBase.div(-2.0), trackWidth.div(-2.0))
      };
  public static final Distance driveBaseRadius =
      Meters.of(Translation2d.kZero.getDistance(moduleTranslations[0]));

  // Drive motor configuration
  public static final Distance wheelRadius = Inches.of(2);
  public static final double wheelRadiusMeters = wheelRadius.in(Meters);
  public static final double driveMotorReduction =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // SDS MK4 L2
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);
  public static final LinearVelocity drivetrainSpeedLimit =
      MetersPerSecond.of(
          0.9
              * (wheelRadiusMeters * 2.0 * Math.PI)
              * KrakenX60Constants.freeSpeed.in(RotationsPerSecond)
              / driveMotorReduction);

  // Chassis movement limits
  private static final LinearVelocity driverSpeedLimit = MetersPerSecond.of(5);
  public static final LinearVelocity maxChassisVelocity =
      MetersPerSecond.of(
          Math.min(
              drivetrainSpeedLimit.in(MetersPerSecond), driverSpeedLimit.in(MetersPerSecond)));
  public static final LinearAcceleration maxChassisAcceleration =
      MetersPerSecondPerSecond.of(3.0);

  public static final AngularVelocity maxChassisAngularVelocity =
      RadiansPerSecond.of(maxChassisVelocity.in(MetersPerSecond) / driveBaseRadius.in(Meters));
  public static final AngularAcceleration maxChassisAngularAcceleration =
      RadiansPerSecondPerSecond.of(30);

  public static final PathConstraints pathFollowingConstraints =
      new PathConstraints(
          maxChassisVelocity.in(MetersPerSecond),
          maxChassisAcceleration.in(MetersPerSecondPerSecond),
          maxChassisAngularVelocity.in(RadiansPerSecond),
          maxChassisAngularAcceleration.in(RadiansPerSecondPerSecond));

  // Turn motor configuration
  public static final boolean turnInverted = false;
  public static final double turnMotorReduction = (32.0 / 15.0) * (60.0 / 10.0); // SDS MK4
  // Every 1 rotation of the azimuth results in coupleRatio drive motor turns
  private static final double coupleRatio = (50.0 / 14.0); // SDS MK4 L2
  public static final DCMotor turnGearbox = DCMotor.getKrakenX60(1);

  // Absolute turn encoder configuration
  public static final boolean turnEncoderInverted = false;

  // PathPlanner configuration
  public static final Mass robotMass = Pounds.of(150);
  public static final MomentOfInertia robotMoi = KilogramSquareMeters.of(6);
  public static final double wheelCof = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMass.in(Kilograms),
          robotMoi.in(KilogramSquareMeters),
          new ModuleConfig(
              wheelRadiusMeters,
              drivetrainSpeedLimit.in(MetersPerSecond),
              wheelCof,
              driveGearbox.withReduction(driveMotorReduction),
              KrakenX60Constants.defaultSupplyCurrentLimit,
              1),
          moduleTranslations);

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs()
          .withKP(300)
          .withKI(0)
          .withKD(1.5)
          .withKS(0.1)
          .withKV(1.91)
          .withKA(0)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0).withKV(0.124);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType steerClosedLoopOutput =
      ClosedLoopOutputType.TorqueCurrentFOC;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType driveClosedLoopOutput =
      ClosedLoopOutputType.TorqueCurrentFOC;

  // The type of motor used for the drive motor
  private static final DriveMotorArrangement driveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  // The type of motor used for the steer motor
  private static final SteerMotorArrangement steerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors
  private static final SteerFeedbackType steerFeedbackType = SteerFeedbackType.FusedCANcoder;

  // TorqueCurrent peak at which the wheels start to slip; used for slip detection in
  // TorqueCurrentFOC control mode. This needs to be tuned to your individual robot.
  static final int slipCurrent = 120;

  // Hardware stator current limit for drive motors
  static final int driveStatorCurrentLimit = KrakenX60Constants.defaultStatorCurrentLimit;

  // Stator current limit for azimuth (steer) motors; lower than drive to reduce brownout risk
  // since steering requires minimal torque compared to driving.
  static final int steerStatorCurrentLimit = 60;

  private static final TalonFXConfiguration driveInitialConfigs =
      new TalonFXConfiguration()
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(slipCurrent)
                  .withPeakReverseTorqueCurrent(-slipCurrent))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(driveStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(KrakenX60Constants.defaultSupplyCurrentLimit)
                  .withSupplyCurrentLimitEnable(true));

  // Azimuth does not require much torque; keep stator limit low to reduce brownout risk
  // since steering requires minimal torque compared to driving.
  private static final TalonFXConfiguration turnInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(steerStatorCurrentLimit)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(KrakenX60Constants.defaultSupplyCurrentLimit)
                  .withSupplyCurrentLimitEnable(true));

  private static final boolean invertLeftSide = false;
  private static final boolean invertRightSide = false;

  // These are only used for simulation
  private static final MomentOfInertia steerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia driveInertia = KilogramSquareMeters.of(0.025);
  // Simulated voltage necessary to overcome friction
  private static final Voltage steerFrictionVoltage = Volts.of(0.2);
  private static final Voltage driveFrictionVoltage = Volts.of(0.2);

  public static final SwerveDrivetrainConstants drivetrainConstants =
      new SwerveDrivetrainConstants().withCANBusName(CANHD.bus.getName());

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(driveMotorReduction)
              .withSteerMotorGearRatio(turnMotorReduction)
              .withCouplingGearRatio(coupleRatio)
              .withWheelRadius(wheelRadius)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSlipCurrent(Amps.of(slipCurrent))
              .withSpeedAt12Volts(drivetrainSpeedLimit)
              .withDriveMotorType(driveMotorType)
              .withSteerMotorType(steerMotorType)
              .withFeedbackSource(steerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(turnInitialConfigs)
              .withSteerInertia(steerInertia)
              .withDriveInertia(driveInertia)
              .withSteerFrictionVoltage(steerFrictionVoltage)
              .withDriveFrictionVoltage(driveFrictionVoltage);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontLeft =
          constantCreator.createModuleConstants(
              CANHD.frontLeftTurn,
              CANHD.frontLeftDrive,
              CANHD.frontLeftTurnAbsEncoder,
              Rotations.of(0),
              wheelBase.div(2.0),
              trackWidth.div(2.0),
              invertLeftSide,
              turnInverted,
              turnEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      frontRight =
          constantCreator.createModuleConstants(
              CANHD.frontRightTurn,
              CANHD.frontRightDrive,
              CANHD.frontRightTurnAbsEncoder,
              Rotations.of(0),
              wheelBase.div(2.0),
              trackWidth.div(-2.0),
              invertRightSide,
              turnInverted,
              turnEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backLeft =
          constantCreator.createModuleConstants(
              CANHD.backLeftTurn,
              CANHD.backLeftDrive,
              CANHD.backLeftTurnAbsEncoder,
              Rotations.of(0),
              wheelBase.div(-2.0),
              trackWidth.div(2.0),
              invertLeftSide,
              turnInverted,
              turnEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      backRight =
          constantCreator.createModuleConstants(
              CANHD.backRightTurn,
              CANHD.backRightDrive,
              CANHD.backRightTurnAbsEncoder,
              Rotations.of(0),
              wheelBase.div(-2.0),
              trackWidth.div(-2.0),
              invertRightSide,
              turnInverted,
              turnEncoderInverted);

  /**
   * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
   * program,.
   */
  //   public static CommandSwerveDrivetrain createDrivetrain() {
  //     return new CommandSwerveDrivetrain(
  //         drivetrainConstants, frontLeft, frontRight, backLeft, backRight);
  //   }

  /** Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types. */
  public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          modules);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set
     *     to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *     [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation The standard deviation for vision calculation in the form [x,
     *     y, theta]ᵀ, with units in meters and radians
     * @param modules Constants for each specific module
     */
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          TalonFX::new,
          TalonFX::new,
          CANcoder::new,
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}
