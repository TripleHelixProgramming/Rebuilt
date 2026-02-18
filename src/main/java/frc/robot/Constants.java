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

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class RobotConstants {
    public static final double kNominalVoltage = 12.0;
  }

  public static final class MotorConstants {
    public static final class NEOConstants {
      public static final AngularVelocity kFreeSpeed = RPM.of(5676);
      public static final int kDefaultSupplyCurrentLimit = 60;
    }

    public static final class NEO550Constants {
      public static final AngularVelocity kFreeSpeed = RPM.of(11000);
      public static final int kDefaultSupplyCurrentLimit = 20;
    }

    public static final class NEOVortexConstants {
      public static final AngularVelocity kFreeSpeed = RPM.of(6784);
      public static final int kDefaultSupplyCurrentLimit = 60;
    }

    public static final class KrakenX60Constants {
      public static final AngularVelocity kFreeSpeed = RPM.of(6000);
      public static final int kDefaultSupplyCurrentLimit = 60;
    }
  }

  public static final class DIOPorts {
    // max length is 8
    public static final int[] autonomousModeSelector = {0, 1, 2};

    public static final int allianceColorSelector = 3;

    public static final int turretAbsEncoder = 4;
  }

  public static final class OIPorts {
    public static final int defaultDriver = 0;
    public static final int defaultOperator = 1;
  }

  public static final class CANBusPorts {

    public static final class CAN2 {
      public static final CANBus bus = CANBus.roboRIO();

      // Drivetrain
      public static final int gyro = 0;

      // Launcher
      public static final int turret = 12;
      public static final int hood = 13;
      public static final int flywheelLeader = 14;
      public static final int flywheelFollower = 15;

      // Feeder
      public static final int spindexer = 16;
      public static final int kicker = 17;

      // Intake
      public static final int intakeRollerLeader = 22;
      public static final int intakeRollerFollower = 23;
    }

    public static final class CANHD {
      // CAN bus that the devices are located on;
      // All swerve devices must share the same CAN bus
      public static final CANBus bus = new CANBus("canivore", "./logs/example.hoot");

      // Drivetrain
      public static final int backLeftDrive = 10;
      public static final int backRightDrive = 18;
      public static final int frontRightDrive = 20;
      public static final int frontLeftDrive = 28;

      public static final int backLeftTurn = 11;
      public static final int backRightTurn = 19;
      public static final int frontRightTurn = 21;
      public static final int frontLeftTurn = 29;

      public static final int backRightTurnAbsEncoder = 31;
      public static final int frontRightTurnAbsEncoder = 33;
      public static final int frontLeftTurnAbsEncoder = 43;
      public static final int backLeftTurnAbsEncoder = 45;
    }
  }
}
