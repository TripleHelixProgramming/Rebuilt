package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;
import frc.robot.Constants.RobotConstants;

public class IntakeConstants {
  public class IntakeRoller {
    public static final Distance rollerRadius = Inches.of(0.75);

    // motor controller
    public static final double motorReduction = 1.0;
    public static final AngularVelocity maxAngularVelocity =
        KrakenX60Constants.kFreeSpeed.div(motorReduction);
    public static final Slot0Configs intakeGains =
        new Slot0Configs()
            .withKP(0.1)
            .withKI(0.0)
            .withKD(0.05)
            .withKS(0.0)
            .withKV(RobotConstants.kNominalVoltage / maxAngularVelocity.in(RotationsPerSecond))
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // simulation
    public static final DCMotor gearbox = DCMotor.getKrakenX60(1);
  }
}
