package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.MotorConstants.KrakenX60Constants;

public class IntakeConstants {
  public class IntakeRoller {
    public static final double gearboxRatio = 1.0;
    public static final Distance rollerRadius = Inches.of(3.0);

    // gains
    public static final Slot0Configs intakeGains =
        new Slot0Configs()
            .withKP(1.0)
            .withKI(0.0)
            .withKD(1.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // motor controller
    public static final double motorReudction = 1.0;
    public static final AngularVelocity maxAngularVelocity =
        KrakenX60Constants.kFreeSpeed.div(motorReudction);

    // simulation
    public static final DCMotor gearbox = DCMotor.getKrakenX60(1);
  }
}
