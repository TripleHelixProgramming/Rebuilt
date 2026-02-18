package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;

public class IntakeConstants {
  public class IntakeRoller {
    public static final Distance rollerRadius = Inches.of(0.85);

    // motor controller
    public static final double motorReduction = 1.0;
    public static final AngularVelocity maxAngularVelocity =
        KrakenX60Constants.kFreeSpeed.div(motorReduction);
    public static final Slot0Configs velocityVoltageGains =
        new Slot0Configs().withKP(0.11).withKI(0.0).withKD(0.0).withKS(0.1).withKV(0.12);
    // .withKV(
    //     RobotConstants.kNominalVoltage
    //         / KrakenX60Constants.kFreeSpeed.in(RotationsPerSecond))
    public static final Slot1Configs velocityTorqueCurrentGains =
        new Slot1Configs().withKP(5).withKI(0.0).withKD(0.0).withKS(2.5);

    // simulation
    public static final DCMotor gearbox = DCMotor.getKrakenX60(2);
  }

  public class PneumaticConstants {
    // hopper
    public static final int hopperForwardChannel = 2;
    public static final int hopperReverseChannel = 3;

    // intake arm
    public static final int intakeArmForwardChannel = 0;
    public static final int intakeArmReverseChannel = 1;
  }
}
