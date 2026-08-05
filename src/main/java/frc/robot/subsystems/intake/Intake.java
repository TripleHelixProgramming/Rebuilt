package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.intake.IntakeConstants.ArmConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final RollerIO upperRollerIO;
  private final RollerIO lowerRollerIO;
  private final IntakeArmIO leftArmIO;
  private final IntakeArmIO rightArmIO;

  private final RollerIOInputsAutoLogged upperRollerInputs = new RollerIOInputsAutoLogged();
  private final RollerIOInputsAutoLogged lowerRollerInputs = new RollerIOInputsAutoLogged();
  private final IntakeArmIOInputsAutoLogged leftArmInputs = new IntakeArmIOInputsAutoLogged();
  private final IntakeArmIOInputsAutoLogged rightArmInputs = new IntakeArmIOInputsAutoLogged();

  private final Alert upperRollerDisconnectedAlert;
  private final Alert lowerRollerDisconnectedAlert;
  private final Alert leftArmDisconnectedAlert;
  private final Alert rightArmDisconnectedAlert;

  // Both arms independently follow the same profiled setpoint; commands only ever move the goal.
  private final TrapezoidProfile armProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(PROFILE_MAX_VELOCITY, PROFILE_MAX_ACCELERATION));
  private State armGoal = new State(minPosRad, 0.0);
  private State armSetpoint = new State(minPosRad, 0.0);

  // Only the left arm's Spark has an absolute encoder wired up. Both arms' relative encoders,
  // and the motion profile itself, are seeded from that one reading the first time it's valid.
  private boolean armSeeded = false;

  // Injected after both subsystems are created to avoid a circular dependency.
  // When set, getDeployCommand() and getReverseCommand() will deploy the hopper first if needed.
  private BooleanSupplier hopperIsDeployed;
  private Supplier<Command> hopperDeployCommand;

  public Intake(
      RollerIO upperRollerIO,
      RollerIO lowerRollerIO,
      IntakeArmIO leftArmIO,
      IntakeArmIO rightArmIO) {
    this.upperRollerIO = upperRollerIO;
    this.lowerRollerIO = lowerRollerIO;
    this.leftArmIO = leftArmIO;
    this.rightArmIO = rightArmIO;

    upperRollerDisconnectedAlert = new Alert("Disconnected upper intake roller", AlertType.kError);
    lowerRollerDisconnectedAlert = new Alert("Disconnected lower intake roller", AlertType.kError);
    leftArmDisconnectedAlert = new Alert("Disconnected intake arm", AlertType.kError);
    rightArmDisconnectedAlert = new Alert("Disconnected intake arm", AlertType.kError);
  }

  @Override
  public void periodic() {
    long t0 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;
    upperRollerIO.updateInputs(upperRollerInputs);
    lowerRollerIO.updateInputs(lowerRollerInputs);
    leftArmIO.updateInputs(leftArmInputs);
    rightArmIO.updateInputs(rightArmInputs);
    long t1 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    Logger.processInputs("UpperRoller", upperRollerInputs);
    Logger.processInputs("LowerRoller", lowerRollerInputs);
    Logger.processInputs("LeftArm", leftArmInputs);
    Logger.processInputs("RightArm", rightArmInputs);
    long t2 = Constants.FeatureFlags.PROFILING_ENABLED ? System.nanoTime() : 0;

    upperRollerDisconnectedAlert.set(!upperRollerInputs.connected);
    lowerRollerDisconnectedAlert.set(!lowerRollerInputs.connected);
    leftArmDisconnectedAlert.set(!leftArmInputs.connected);
    rightArmDisconnectedAlert.set(!rightArmInputs.connected);
    Logger.recordOutput("Faults/Intake/UpperRollerDisconnected", !upperRollerInputs.connected);
    Logger.recordOutput("Faults/Intake/LowerRollerDisconnected", !lowerRollerInputs.connected);
    Logger.recordOutput("Faults/Intake/LeftArmDisconnected", !leftArmInputs.connected);
    Logger.recordOutput("Faults/Intake/RightArmDisconnected", !rightArmInputs.connected);

    // Seed both relative encoders, and the motion profile, from the left arm's absolute encoder
    // once it reports connected. Runs once at boot.
    if (!armSeeded && leftArmInputs.connected) {
      double seedPositionRad = leftArmInputs.absolutePosition.getRadians();
      leftArmIO.resetEncoder(Radians.of(seedPositionRad));
      rightArmIO.resetEncoder(Radians.of(seedPositionRad));
      armGoal = new State(seedPositionRad, 0.0);
      armSetpoint = new State(seedPositionRad, 0.0);
      armSeeded = true;
    }

    // Advance the arm motion profile and drive both arms to the resulting setpoint. Commands
    // never set arm position directly — they only move armGoal, and this is the sole place
    // setPosition() is called.
    armSetpoint = armProfile.calculate(Robot.defaultPeriodSecs, armSetpoint, armGoal);
    leftArmIO.setPosition(
        Radians.of(armSetpoint.position), RadiansPerSecond.of(armSetpoint.velocity));
    rightArmIO.setPosition(
        Radians.of(armSetpoint.position), RadiansPerSecond.of(armSetpoint.velocity));

    // Profiling output
    if (Constants.FeatureFlags.PROFILING_ENABLED) {
      long totalMs = (t2 - t0) / 1_000_000;
      if (totalMs > 2) {
        System.out.println(
            "[Intake] update="
                + (t1 - t0) / 1_000_000
                + "ms log="
                + (t2 - t1) / 1_000_000
                + "ms total="
                + totalMs
                + "ms");
      }
    }
  }

  public void stop() {
    upperRollerIO.setOpenLoop(Volts.of(0.0));
    lowerRollerIO.setOpenLoop(Volts.of(0.0));
    armGoal = new State(minPosRad, 0.0);
  }

  public void deployArm() {
    armGoal = new State(maxPosRad, 0.0);
  }

  public void retractArm() {
    armGoal = new State(minPosRad, 0.0);
  }

  public boolean isStowed() {
    return false;
  }

  /**
   * Configures the hopper deploy interlock for getDeployCommand() and getReverseCommand(). Must be
   * called after both the Intake and Hopper subsystems are created.
   *
   * @param hopperIsDeployed supplier returning true when the hopper is deployed
   * @param hopperDeployCommand factory that returns a fresh command to deploy the hopper
   */
  public void setDeployInterlock(
      BooleanSupplier hopperIsDeployed, Supplier<Command> hopperDeployCommand) {
    this.hopperIsDeployed = hopperIsDeployed;
    this.hopperDeployCommand = hopperDeployCommand;
  }

  public Command getStopCommand() {
    return Commands.startEnd(this::stop, () -> {}, this).withName("Retract and stop");
  }

  @Override
  public Command getDefaultCommand() {
    return getStopCommand();
  }

  public Command getDeployCommand() {
    return Commands.sequence(
            hopperInterlock(),
            Commands.runOnce(this::deployArm, this),
            this.idle().withTimeout(0.5),
            Commands.startEnd(
                () -> {
                  upperRollerIO.setVelocity(MetersPerSecond.of(6.0));
                  lowerRollerIO.setVelocity(MetersPerSecond.of(6.0));
                },
                () -> {},
                this))
        .withName("Intake");
  }

  /** Returns the total motor current draw for battery simulation. */
  public double getSimCurrentDrawAmps() {
    return upperRollerInputs.currentAmps
        + lowerRollerInputs.currentAmps
        + leftArmInputs.currentAmps
        + rightArmInputs.currentAmps;
  }

  public Command getReverseCommand() {
    return Commands.sequence(
            hopperInterlock(),
            Commands.runOnce(this::deployArm, this),
            this.idle().withTimeout(0.5),
            Commands.startEnd(
                () -> {
                  upperRollerIO.setVelocity(MetersPerSecond.of(-4.0));
                  lowerRollerIO.setVelocity(MetersPerSecond.of(-4.0));
                },
                () -> {},
                this))
        .withName("Reverse");
  }

  /** Returns the hopper deploy interlock step, or a no-op if no interlock has been configured. */
  private Command hopperInterlock() {
    if (hopperIsDeployed == null) {
      return Commands.none();
    }
    return Commands.either(Commands.none(), hopperDeployCommand.get(), hopperIsDeployed);
  }

  public Command getShakeIntakeCommand() {
    return Commands.sequence(
            this.idle().withTimeout(1.0),
            Commands.runOnce(this::stop, this),
            this.idle().withTimeout(1.0),
            Commands.runOnce(this::deployArm, this))
        .repeatedly();
  }
}
