// Copyright 2021-2024 FRC 6328
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

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CANBusPorts.CAN2;
import frc.robot.util.CanandgyroThread;
import frc.robot.util.CanandgyroThread.GyroInputs;
import java.util.Queue;

/** IO implementation for Redux Canandgyro. */
public class GyroIOBoron implements GyroIO {
  private final Canandgyro canandgyro;
  private final GyroInputs gyroInputs;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> yawPositionQueue;

  public GyroIOBoron() {
    canandgyro = new Canandgyro(CAN2.gyro);
    gyroInputs = CanandgyroThread.getInstance().registerCanandgyro(canandgyro);
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(canandgyro::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // Read from cached values (non-blocking) - updated by CanandgyroThread
    inputs.connected = gyroInputs.isConnected();
    inputs.calibrated = !gyroInputs.isCalibrating();
    inputs.yawPosition = Rotation2d.fromRotations(gyroInputs.getYaw());
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(gyroInputs.getAngularVelocityYaw());

    inputs.odometryYawTimestamps = new double[yawTimestampQueue.size()];
    for (int i = 0; i < inputs.odometryYawTimestamps.length; i++) {
      inputs.odometryYawTimestamps[i] = yawTimestampQueue.poll();
    }
    inputs.odometryYawPositions = new Rotation2d[yawPositionQueue.size()];
    for (int i = 0; i < inputs.odometryYawPositions.length; i++) {
      inputs.odometryYawPositions[i] = Rotation2d.fromRotations(yawPositionQueue.poll());
    }
  }
}
