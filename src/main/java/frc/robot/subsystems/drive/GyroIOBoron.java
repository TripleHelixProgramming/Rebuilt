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
import java.util.Queue;

/** IO implementation for NavX. */
public class GyroIOBoron implements GyroIO {
  private Canandgyro canandgyro;
  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> yawPositionQueue;

  public GyroIOBoron() {
    canandgyro = new Canandgyro(CAN2.gyro);
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(canandgyro::getYaw);
  }

  // Check if gyro is calibrated
  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = canandgyro.isConnected();
    inputs.calibrated = !canandgyro.isCalibrating();
    inputs.yawPosition = Rotation2d.fromRotations(canandgyro.getYaw());
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(canandgyro.getAngularVelocityYaw());

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
