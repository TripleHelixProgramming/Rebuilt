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
import static frc.robot.subsystems.drive.DriveConstants.*;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.FeatureFlags;
import frc.robot.Constants.Mode;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final double ODOMETRY_FREQUENCY =
      new CANBus(DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator visionPose =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
  private boolean poseAsserted = false;

  private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();
  private final SwerveModuleState[] emptyModuleStates = new SwerveModuleState[] {};
  // Pre-allocated for getModuleStates()/getModulePositions() to avoid array allocation each call
  private final SwerveModuleState[] measuredStates = new SwerveModuleState[4];
  private final SwerveModulePosition[] measuredPositions = new SwerveModulePosition[4];
  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  // Pre-allocated to avoid allocations in odometry loop - fields are mutated in place
  private final SwerveModulePosition[] moduleDeltas =
      new SwerveModulePosition[] {
        new SwerveModulePosition(), new SwerveModulePosition(),
        new SwerveModulePosition(), new SwerveModulePosition()
      };
  private ChassisSpeeds chassisSpeeds;

  // PID controllers for following Choreo trajectories
  private final PIDController xController = new PIDController(8.01, 0.0, 0.0);
  private final PIDController yController = new PIDController(8.01, 0.0, 0.0);
  private final PIDController headingController = new PIDController(8.01, 0.0, 0.0);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getRobotRelativeChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    long startNanos = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    odometryLock.lock(); // Prevents odometry updates while reading data
    long t1 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    gyroIO.updateInputs(gyroInputs);
    long t2 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    Logger.processInputs("Drive/Gyro", gyroInputs);
    long t3 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    for (var module : modules) {
      module.periodic();
    }
    long t4 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", emptyModuleStates);
      Logger.recordOutput("SwerveStates/SetpointsOptimized", emptyModuleStates);
    }
    long t5 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    Logger.recordOutput("Drive/sampleCount", sampleCount);
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        // Mutate pre-allocated delta objects to avoid allocations
        moduleDeltas[moduleIndex].distanceMeters =
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters;
        moduleDeltas[moduleIndex].angle = modulePositions[moduleIndex].angle;
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      visionPose.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
    }
    long t6 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Update gyro alert
    boolean gyroDisconnected = !gyroInputs.connected && Constants.currentMode != Mode.SIM;
    gyroDisconnectedAlert.set(gyroDisconnected);
    Logger.recordOutput("Faults/Drive/GyroDisconnected", gyroDisconnected);

    // Profiling output
    if (FeatureFlags.PROFILING_ENABLED) {
      long totalMs = (t6 - startNanos) / 1_000_000;
      if (totalMs > 5) {
        System.out.println(
            "[Drive] lock="
                + (t1 - startNanos) / 1_000_000
                + "ms gyroUpdate="
                + (t2 - t1) / 1_000_000
                + "ms gyroLog="
                + (t3 - t2) / 1_000_000
                + "ms modules="
                + (t4 - t3) / 1_000_000
                + "ms disabled="
                + (t5 - t4) / 1_000_000
                + "ms odometry="
                + (t6 - t5) / 1_000_000
                + "ms total="
                + totalMs
                + "ms");
      }
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {

    // 1️: Convert continuous speeds to module states
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);
    Logger.recordOutput("SwerveStates/Setpoints", states);

    // 2: Desaturate (apply wheel limits FIRST)
    SwerveDriveKinematics.desaturateWheelSpeeds(states, drivetrainSpeedLimit.in(MetersPerSecond));

    // 3: Reconstruct the ACTUAL chassis speeds after limiting
    ChassisSpeeds limitedSpeeds = kinematics.toChassisSpeeds(states);

    // 4: Now discretize the LIMITED speeds
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(limitedSpeeds, 0.02);

    // 5: Convert discretized speeds back to module states
    SwerveModuleState[] finalStates = kinematics.toSwerveModuleStates(discreteSpeeds);

    // (Optional but usually unnecessary)
    // desaturate again for safety
    SwerveDriveKinematics.desaturateWheelSpeeds(
        finalStates, drivetrainSpeedLimit.in(MetersPerSecond));

    // 6: Send to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(finalStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", finalStates);
  }

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

    // Apply the generated speeds
    runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation()));
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(ZERO_SPEEDS);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }
    return measuredStates;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < 4; i++) {
      measuredPositions[i] = modules[i].getPosition();
    }
    return measuredPositions;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return chassisSpeeds;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current vision pose. */
  @AutoLogOutput(key = "Drive/Pose")
  public Pose2d getPose() {
    return visionPose.getEstimatedPosition();
  }

  /**
   * Returns whether the pose estimator has been authoritatively initialized by an auto routine.
   * Before initialization, the gyro heading is robot-relative and should not be used for
   * field-relative validation such as the vision yawConsistency test.
   */
  public boolean isPoseAsserted() {
    return poseAsserted;
  }

  public Rotation2d getRawGyroRotation() {
    return rawGyroRotation;
  }

  /** Resets the pose estimator to the given pose. */
  public void setPose(Pose2d pose) {
    visionPose.resetPosition(rawGyroRotation, getModulePositions(), pose);
    poseAsserted = true;
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // Track the latest vision estimate while disabled, before the pose is authoritatively
    // initialized by an auto routine. Does not set poseAsserted; yawConsistency is gated
    // in Vision.java on poseAsserted or accumulated accepted-observation count.
    if (!poseAsserted && RobotState.isDisabled()) {
      visionPose.resetPosition(rawGyroRotation, getModulePositions(), visionRobotPoseMeters);
    }

    visionPose.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxChassisVelocity.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxChassisAngularVelocity.in(RadiansPerSecond);
  }

  /** Returns the total motor current draw across all modules for battery simulation. */
  public double getSimCurrentDrawAmps() {
    double total = 0.0;
    for (var module : modules) {
      total += module.getSimCurrentDrawAmps();
    }
    return total;
  }

  public void zeroAbsoluteEncoders() {
    for (var module : modules) {
      module.setTurnZero();
    }
  }
}
