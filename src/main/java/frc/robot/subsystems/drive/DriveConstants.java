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

  public static final String ZERO_ROTATION_KEY = "ZeroRotation";

  // Robot physical dimensions
  public static final Distance WHEEL_BASE = Inches.of(22.5);
  public static final Distance TRACK_WIDTH = Inches.of(19.5);
  public static final Translation2d[] MODULE_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(WHEEL_BASE.div(2.0), TRACK_WIDTH.div(2.0)),
        new Translation2d(WHEEL_BASE.div(2.0), TRACK_WIDTH.div(-2.0)),
        new Translation2d(WHEEL_BASE.div(-2.0), TRACK_WIDTH.div(2.0)),
        new Translation2d(WHEEL_BASE.div(-2.0), TRACK_WIDTH.div(-2.0))
      };
  public static final Distance DRIVE_BASE_RADIUS =
      Meters.of(Translation2d.kZero.getDistance(MODULE_TRANSLATIONS[0]));

  // Drive motor configuration
  public static final Distance WHEEL_RADIUS = Inches.of(2);
  public static final double WHEEL_RADIUS_METERS = WHEEL_RADIUS.in(Meters);
  public static final double DRIVE_MOTOR_REDUCTION =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // SDS MK4 L2
  public static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  public static final LinearVelocity DRIVETRAIN_SPEED_LIMIT =
      MetersPerSecond.of(
          0.9
              * (WHEEL_RADIUS_METERS * 2.0 * Math.PI)
              * DRIVE_GEARBOX.freeSpeedRadPerSec
              / (2.0 * Math.PI)
              / DRIVE_MOTOR_REDUCTION);

  // Chassis movement limits
  private static final LinearVelocity DRIVER_SPEED_LIMIT = MetersPerSecond.of(5);
  public static final LinearVelocity MAX_CHASSIS_VELOCITY =
      MetersPerSecond.of(
          Math.min(
              DRIVETRAIN_SPEED_LIMIT.in(MetersPerSecond), DRIVER_SPEED_LIMIT.in(MetersPerSecond)));
  public static final LinearAcceleration MAX_CHASSIS_ACCELERATION =
      MetersPerSecondPerSecond.of(3.0);

  public static final AngularVelocity MAX_CHASSIS_ANGULAR_VELOCITY =
      RadiansPerSecond.of(MAX_CHASSIS_VELOCITY.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));
  public static final AngularAcceleration MAX_CHASSIS_ANGULAR_ACCELERATION =
      RadiansPerSecondPerSecond.of(30);

  public static final PathConstraints PATH_FOLLOWING_CONSTRAINTS =
      new PathConstraints(
          MAX_CHASSIS_VELOCITY.in(MetersPerSecond),
          MAX_CHASSIS_ACCELERATION.in(MetersPerSecondPerSecond),
          MAX_CHASSIS_ANGULAR_VELOCITY.in(RadiansPerSecond),
          MAX_CHASSIS_ANGULAR_ACCELERATION.in(RadiansPerSecondPerSecond));

  // Turn motor configuration
  public static final boolean TURN_INVERTED = false;
  public static final double TURN_MOTOR_REDUCTION = (32.0 / 15.0) * (60.0 / 10.0); // SDS MK4
  // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns
  private static final double COUPLE_RATIO = (50.0 / 14.0); // SDS MK4 L2
  public static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  // Absolute turn encoder configuration
  public static final boolean TURN_ENCODER_INVERTED = false;

  // PathPlanner configuration
  public static final Mass ROBOT_MASS = Pounds.of(150);
  public static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6);
  public static final double WHEEL_COF = 1.2;
  public static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS.in(Kilograms),
          ROBOT_MOI.in(KilogramSquareMeters),
          new ModuleConfig(
              WHEEL_RADIUS_METERS,
              DRIVETRAIN_SPEED_LIMIT.in(MetersPerSecond),
              WHEEL_COF,
              DRIVE_GEARBOX.withReduction(DRIVE_MOTOR_REDUCTION),
              KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT,
              1),
          MODULE_TRANSLATIONS);

  // The steer motor uses any SwerveModule.SteerRequestType control request with the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs STEER_GAINS =
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
  private static final Slot0Configs DRIVE_GAINS =
      new Slot0Configs().withKP(10).withKI(0).withKD(0).withKS(0).withKV(0.124);

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
      ClosedLoopOutputType.TorqueCurrentFOC;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
      ClosedLoopOutputType.TorqueCurrentFOC;

  // The type of motor used for the drive motor
  private static final DriveMotorArrangement DRIVE_MOTOR_TYPE =
      DriveMotorArrangement.TalonFX_Integrated;
  // The type of motor used for the steer motor
  private static final SteerMotorArrangement STEER_MOTOR_TYPE =
      SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors
  private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

  // TorqueCurrent peak at which the wheels start to slip; used for slip detection in
  // TorqueCurrentFOC control mode. This needs to be tuned to your individual robot.
  static final int SLIP_CURRENT = 120;

  // Hardware stator current limit for drive motors
  static final int DRIVE_STATOR_CURRENT_LIMIT = KrakenX60Constants.DEFAULT_STATOR_CURRENT_LIMIT;

  // Stator current limit for azimuth (steer) motors; lower than drive to reduce brownout risk
  // since steering requires minimal torque compared to driving.
  static final int STEER_STATOR_CURRENT_LIMIT = 60;

  private static final TalonFXConfiguration DRIVE_INITIAL_CONFIGS =
      new TalonFXConfiguration()
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(SLIP_CURRENT)
                  .withPeakReverseTorqueCurrent(-SLIP_CURRENT))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true));

  // Azimuth does not require much torque; keep stator limit low to reduce brownout risk
  // since steering requires minimal torque compared to driving.
  private static final TalonFXConfiguration TURN_INITIAL_CONFIGS =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(STEER_STATOR_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(KrakenX60Constants.DEFAULT_SUPPLY_CURRENT_LIMIT)
                  .withSupplyCurrentLimitEnable(true));

  private static final boolean INVERT_LEFT_SIDE = false;
  private static final boolean INVERT_RIGHT_SIDE = false;

  // These are only used for simulation
  private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.025);
  // Simulated voltage necessary to overcome friction
  private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
  private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

  public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
      new SwerveDrivetrainConstants().withCANBusName(CANHD.BUS.getName());

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      CONSTANT_CREATOR =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(DRIVE_MOTOR_REDUCTION)
              .withSteerMotorGearRatio(TURN_MOTOR_REDUCTION)
              .withCouplingGearRatio(COUPLE_RATIO)
              .withWheelRadius(WHEEL_RADIUS)
              .withSteerMotorGains(STEER_GAINS)
              .withDriveMotorGains(DRIVE_GAINS)
              .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
              .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
              .withSlipCurrent(Amps.of(SLIP_CURRENT))
              .withSpeedAt12Volts(DRIVETRAIN_SPEED_LIMIT)
              .withDriveMotorType(DRIVE_MOTOR_TYPE)
              .withSteerMotorType(STEER_MOTOR_TYPE)
              .withFeedbackSource(STEER_FEEDBACK_TYPE)
              .withDriveMotorInitialConfigs(DRIVE_INITIAL_CONFIGS)
              .withSteerMotorInitialConfigs(TURN_INITIAL_CONFIGS)
              .withSteerInertia(STEER_INERTIA)
              .withDriveInertia(DRIVE_INERTIA)
              .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
              .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_LEFT =
          CONSTANT_CREATOR.createModuleConstants(
              CANHD.FRONT_LEFT_TURN,
              CANHD.FRONT_LEFT_DRIVE,
              CANHD.FRONT_LEFT_TURN_ABS_ENC,
              Rotations.of(0),
              WHEEL_BASE.div(2.0),
              TRACK_WIDTH.div(2.0),
              INVERT_LEFT_SIDE,
              TURN_INVERTED,
              TURN_ENCODER_INVERTED);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_RIGHT =
          CONSTANT_CREATOR.createModuleConstants(
              CANHD.FRONT_RIGHT_TURN,
              CANHD.FRONT_RIGHT_DRIVE,
              CANHD.FRONT_RIGHT_TURN_ABS_ENC,
              Rotations.of(0),
              WHEEL_BASE.div(2.0),
              TRACK_WIDTH.div(-2.0),
              INVERT_RIGHT_SIDE,
              TURN_INVERTED,
              TURN_ENCODER_INVERTED);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_LEFT =
          CONSTANT_CREATOR.createModuleConstants(
              CANHD.BACK_LEFT_TURN,
              CANHD.BACK_LEFT_DRIVE,
              CANHD.BACK_LEFT_TURN_ABS_ENC,
              Rotations.of(0),
              WHEEL_BASE.div(-2.0),
              TRACK_WIDTH.div(2.0),
              INVERT_LEFT_SIDE,
              TURN_INVERTED,
              TURN_ENCODER_INVERTED);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_RIGHT =
          CONSTANT_CREATOR.createModuleConstants(
              CANHD.BACK_RIGHT_TURN,
              CANHD.BACK_RIGHT_DRIVE,
              CANHD.BACK_RIGHT_TURN_ABS_ENC,
              Rotations.of(0),
              WHEEL_BASE.div(-2.0),
              TRACK_WIDTH.div(-2.0),
              INVERT_RIGHT_SIDE,
              TURN_INVERTED,
              TURN_ENCODER_INVERTED);

  /**
   * Creates a CommandSwerveDrivetrain instance. This should only be called once in your robot
   * program,.
   */
  //   public static CommandSwerveDrivetrain createDrivetrain() {
  //     return new CommandSwerveDrivetrain(
  //         DRIVETRAIN_CONSTANTS, FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);
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
