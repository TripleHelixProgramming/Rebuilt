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

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.FeatureFlags;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final String name;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, String name) {
    this.io = io;
    this.name = name;
    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on module " + name + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on module " + name + ".", AlertType.kError);

    // Set turn zero from preferences
    Rotation2d turnZeroFromCancoder = inputs.turnZero;
    Preferences.initDouble(ZERO_ROTATION_KEY + name, turnZeroFromCancoder.getRadians());
    Rotation2d turnZeroFromPreferences =
        new Rotation2d(
            Preferences.getDouble(ZERO_ROTATION_KEY + name, turnZeroFromCancoder.getRadians()));
    io.setTurnZero(turnZeroFromPreferences);
    Logger.recordOutput(
        "Drive/Module" + name + "/TurnZeroRad", turnZeroFromPreferences.getRadians());
  }

  public void periodic() {
    long t0 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    io.updateInputs(inputs);
    long t1 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    Logger.processInputs("Drive/Module" + name, inputs);
    long t2 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * WHEEL_RADIUS_METERS;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    Logger.recordOutput("Faults/Module" + name + "/DriveDisconnected", !inputs.driveConnected);
    Logger.recordOutput("Faults/Module" + name + "/TurnDisconnected", !inputs.turnConnected);
    long t3 = FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    // Profiling output
    if (FeatureFlags.PROFILING_ENABLED) {
      long totalMs = (t3 - t0) / 1_000_000;
      if (totalMs > 2) {
        System.out.println(
            "[Module"
                + name
                + "] updateInputs="
                + (t1 - t0) / 1_000_000
                + "ms log="
                + (t2 - t1) / 1_000_000
                + "ms rest="
                + (t3 - t2) / 1_000_000
                + "ms");
      }
    }
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / WHEEL_RADIUS_METERS);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * WHEEL_RADIUS_METERS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * WHEEL_RADIUS_METERS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  /** Returns the total motor current draw for battery simulation. */
  public double getSimCurrentDrawAmps() {
    return inputs.driveCurrentAmps + inputs.turnCurrentAmps;
  }

  /** Sets the zero position of the turn axis to the current rotation */
  public void setTurnZero() {
    Rotation2d newTurnZero = inputs.turnZero.minus(inputs.turnPosition);
    io.setTurnZero(newTurnZero);
    Preferences.setDouble(ZERO_ROTATION_KEY + name, newTurnZero.getRadians());
    Logger.recordOutput("Drive/Module" + name + "/TurnZeroRad", newTurnZero.getRadians());
  }
}
