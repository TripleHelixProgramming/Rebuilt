package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.game.Field;
import frc.game.GameState;
import frc.lib.AllianceSelector;
import frc.lib.AutoOption;
import frc.lib.AutoSelector;
import frc.lib.CommandZorroController;
import frc.lib.ControllerSelector;
import frc.lib.ControllerSelector.ControllerConfig;
import frc.lib.ControllerSelector.ControllerFunction;
import frc.lib.ControllerSelector.ControllerType;
import frc.robot.Constants.DIOPorts;
import frc.robot.auto.B_MoveForward1M;
import frc.robot.auto.B_Path;
import frc.robot.auto.R_MoveAndRotate;
import frc.robot.auto.R_MoveStraight;
import frc.robot.auto.TraversingTheBump;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOBoron;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSimWPI;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.KickerIO;
import frc.robot.subsystems.feeder.KickerIOSimSpark;
import frc.robot.subsystems.feeder.KickerIOSpark;
import frc.robot.subsystems.feeder.SpindexerIO;
import frc.robot.subsystems.feeder.SpindexerIOSimSpark;
import frc.robot.subsystems.feeder.SpindexerIOSpark;
import frc.robot.subsystems.intake.HopperIO;
import frc.robot.subsystems.intake.HopperIOReal;
import frc.robot.subsystems.intake.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeArmIO;
import frc.robot.subsystems.intake.IntakeArmIOReal;
import frc.robot.subsystems.intake.IntakeArmIOSim;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOSimTalonFX;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.launcher.FlywheelIO;
import frc.robot.subsystems.launcher.FlywheelIOSimTalonFX;
import frc.robot.subsystems.launcher.FlywheelIOSimWPI;
import frc.robot.subsystems.launcher.HoodIO;
import frc.robot.subsystems.launcher.HoodIOSimSpark;
import frc.robot.subsystems.launcher.HoodIOSpark;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.launcher.TurretIO;
import frc.robot.subsystems.launcher.TurretIOSimSpark;
import frc.robot.subsystems.launcher.TurretIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.CanandgyroThread;
import frc.robot.util.SparkOdometryThread;
import frc.robot.util.VisionThread;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  public static final AllianceSelector allianceSelector =
      new AllianceSelector(DIOPorts.allianceColorSelector);
  private final AutoSelector autoSelector =
      new AutoSelector(DIOPorts.autonomousModeSelector, allianceSelector::getAllianceColor);
  public static final Field2d field = new Field2d();

  public final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  // Subsystems
  private Drive drive;
  private Vision vision;
  private Launcher launcher;
  private Feeder feeder;
  private Intake intake;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL: // Running on a real robot
        // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

        // Instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOBoron(),
                new ModuleIOTalonFX(DriveConstants.FrontLeft),
                new ModuleIOTalonFX(DriveConstants.FrontRight),
                new ModuleIOTalonFX(DriveConstants.BackLeft),
                new ModuleIOTalonFX(DriveConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVisionSim(
                    cameraFrontRightName, robotToFrontRightCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    cameraFrontLeftName, robotToFrontLeftCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    cameraBackRightName, robotToBackRightCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    cameraBackLeftName, robotToBackLeftCamera, drive::getPose));
        launcher =
            new Launcher(
                drive::getPose,
                drive::getRobotRelativeChassisSpeeds,
                new TurretIOSpark(),
                new FlywheelIOSimWPI(),
                new HoodIOSpark());
        intake = new Intake(new IntakeRollerIOTalonFX(), new IntakeArmIOReal(), new HopperIOReal());
        feeder = new Feeder(new SpindexerIOSpark(), new KickerIOSpark());
        break;

      case SIM: // Running a physics simulator
        // Log to NT
        Logger.addDataReceiver(new NT4Publisher());

        // Instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSimWPI(DriveConstants.FrontLeft),
                new ModuleIOSimWPI(DriveConstants.FrontRight),
                new ModuleIOSimWPI(DriveConstants.BackLeft),
                new ModuleIOSimWPI(DriveConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVisionSim(
                    cameraFrontRightName, robotToFrontRightCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    cameraFrontLeftName, robotToFrontLeftCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    cameraBackRightName, robotToBackRightCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    cameraBackLeftName, robotToBackLeftCamera, drive::getPose));
        launcher =
            new Launcher(
                drive::getPose,
                drive::getRobotRelativeChassisSpeeds,
                new TurretIOSimSpark(),
                new FlywheelIOSimTalonFX(),
                new HoodIOSimSpark());
        feeder = new Feeder(new SpindexerIOSimSpark(), new KickerIOSimSpark());
        intake =
            new Intake(new IntakeRollerIOSimTalonFX(), new IntakeArmIOSim(), new HopperIOSim());
        break;

      case REPLAY: // Replaying a log
      default:
        // Set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));

        // Disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        launcher =
            new Launcher(
                drive::getPose,
                drive::getRobotRelativeChassisSpeeds,
                new TurretIO() {},
                new FlywheelIO() {},
                new HoodIO() {});
        intake = new Intake(new IntakeRollerIO() {}, new IntakeArmIO() {}, new HopperIO() {});
        feeder = new Feeder(new SpindexerIO() {}, new KickerIO() {});
        break;
    }

    // Start background threads (for non-blocking CAN/network reads)
    SparkOdometryThread.getInstance().start();
    VisionThread.getInstance().start();
    CanandgyroThread.getInstance().start();

    // Start AdvantageKit logger
    Logger.start();

    configureControlPanelBindings();
    configureAutoOptions();

    SmartDashboard.putData(
        "Align Encoders",
        new InstantCommand(() -> drive.zeroAbsoluteEncoders()).ignoringDisable(true));
    SmartDashboard.putData(CommandScheduler.getInstance());
    SmartDashboard.putData(drive);
    SmartDashboard.putData(vision);
    SmartDashboard.putData(launcher);
    SmartDashboard.putData(feeder);
    SmartDashboard.putData(intake);
    SmartDashboard.putData("Field", field);
    Field.plotRegions();

    feeder.setDefaultCommand(
        Commands.startEnd(feeder::stop, () -> {}, feeder).withName("Stop feeder"));
    intake.setDefaultCommand(
        Commands.startEnd(intake::stop, () -> {}, intake).withName("Stop intake"));
    launcher.setDefaultCommand(
        launcher.initializeHoodCommand(
            () -> launcher.aim(GameState.getTarget(drive.getPose()).getTranslation())));
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    GameState.logValues();

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    allianceSelector.disabledPeriodic();
    autoSelector.disabledPeriodic();
    ControllerSelector.getInstance().scan(false);
  }

  /** This function is called once when autonomous mode is enabled. */
  @Override
  public void autonomousInit() {
    drive.setDefaultCommand(Commands.runOnce(drive::stop, drive).withName("Stop"));
    autoSelector.scheduleAuto();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop mode is enabled. */
  @Override
  public void teleopInit() {
    autoSelector.cancelAuto();
    ControllerSelector.getInstance().scan(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void configureControlPanelBindings() {
    ControllerSelector.configure(
        // ZORRO is always preferred as driver in REAL and SIM mode
        new ControllerConfig(
            ControllerFunction.DRIVER,
            ControllerType.ZORRO,
            this::bindZorroDriver,
            Constants.Mode.REAL,
            Constants.Mode.SIM),
        // XBOX is always preferred as operator in REAL and SIM mode
        new ControllerConfig(
            ControllerFunction.OPERATOR,
            ControllerType.XBOX,
            this::bindXboxOperator,
            Constants.Mode.REAL,
            Constants.Mode.SIM),
        // XBOX is permitted as driver in REAL and SIM mode
        new ControllerConfig(
            ControllerFunction.DRIVER,
            ControllerType.XBOX,
            this::bindXboxDriver,
            Constants.Mode.REAL,
            Constants.Mode.SIM));
  }

  public void bindZorroDriver(int port) {
    var zorroDriver = new CommandZorroController(port);

    // Drive in field-relative mode while switch E is up
    // Drive in robot-relative mode while switch E is down
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -zorroDriver.getRightYAxis(),
            () -> -zorroDriver.getRightXAxis(),
            () -> -zorroDriver.getLeftXAxis(),
            () -> zorroDriver.getHID().getEUp(),
            allianceSelector::fieldRotated));

    // Reset gyro to 0° when button G is pressed
    zorroDriver
        .GIn()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetHeading(
                            allianceSelector.fieldRotated()
                                ? Rotation2d.k180deg
                                : Rotation2d.kZero),
                    drive)
                .ignoringDisable(true));

    // Aim at hub
    // zorroDriver
    //     .AIn()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtFixedOrientation(
    //             drive,
    //             () -> -zorroDriver.getRightYAxis(),
    //             () -> -zorroDriver.getRightXAxis(),
    //             () ->
    //                 GameState.getTarget(drive.getPose())
    //                     .toPose2d()
    //                     .getTranslation()
    //                     .minus(drive.getPose().getTranslation())
    //                     .getAngle(),
    //             allianceSelector::fieldRotated));

    // Index
    zorroDriver
        .AIn()
        .whileTrue(Commands.startEnd(feeder::spinForward, () -> {}, feeder).withName("Indexing"));

    // Switch to X pattern when button D is pressed
    // zorroDriver.DIn().onTrue(Commands.runOnce(drive::stopWithX, drive));

    zorroDriver
        .DIn()
        .whileTrue(Commands.startEnd(intake::intakeFuel, () -> {}, intake).withName("Intaking"));
  }

  public void bindXboxDriver(int port) {
    var xboxDriver = new CommandXboxController(port);

    // Drive in field-relative mode while left bumper is released
    // Drive in robot-relative mode while left bumper is pressed
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -xboxDriver.getLeftY(),
            () -> -xboxDriver.getLeftX(),
            () -> -xboxDriver.getRightX(),
            () -> !xboxDriver.getHID().getLeftBumperButton(),
            allianceSelector::fieldRotated));

    // Reset gyro to 0° when B button is pressed
    xboxDriver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetHeading(
                            allianceSelector.fieldRotated()
                                ? Rotation2d.k180deg
                                : Rotation2d.kZero),
                    drive)
                .ignoringDisable(true));

    // xboxDriver
    //     .a()
    //     .whileTrue(
    //         Commands.run(() -> launcher.aim(GameState.getMyHubPose().getTranslation()), launcher)
    //             .withName("Aim at hub"));

    // xboxDriver
    //     .y()
    //     .whileTrue(
    //         Commands.run(() -> launcher.aim(GameState.getFieldTarget().getTranslation()),
    // launcher)
    //             .withName("Aim at Target"));

    // // Point at Hub while A button is held
    // xboxDriver
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtFixedOrientation(
    //             drive,
    //             () -> -xboxDriver.getLeftY(),
    //             () -> -xboxDriver.getLeftX(),
    //             () ->
    //                 GameState.getMyHubPose()
    //                     .toPose2d()
    //                     .getTranslation()
    //                     .minus(drive.getPose().getTranslation())
    //                     .getAngle(),
    //             allianceSelector::fieldRotated));

    // // Point in the direction of the commanded translation while Y button is held
    // xboxDriver
    //     .y()
    //     .whileTrue(
    //         DriveCommands.joystickDrivePointedForward(
    //             drive,
    //             () -> -xboxDriver.getLeftY(),
    //             () -> -xboxDriver.getLeftX(),
    //             allianceSelector::fieldRotated));

    // Point at vision target while A button is held
    // xboxDriver
    //     .a()
    //     .whileTrue(
    //         DriveCommands.pointAtTarget(
    //             drive, () -> vision.getTargetX(0), allianceSelector::fieldRotated));

    // Drive 1m forward while A button is held
    // xboxDriver.a().whileTrue(PathCommands.advanceForward(drive, Meters.of(1)));

    // Align with pose, approaching in correct orientation from 1 m away
    // xboxDriver
    //     .a()
    //     .whileTrue(
    //         PathCommands.dockToTargetPose(
    //             drive, new Pose2d(8.2296, 4.1148, Rotation2d.kZero), Meters.of(1)));

    // Drive to point, approaching in correct orientation from 2 m away
    // xboxDriver
    //  .a()
    // .whileTrue(
    //     PathCommands.dockToTargetPoint(drive, new Translation2d(8.2296, 4.1148), Meters.of(2)));

    // Switch to X pattern when X button is pressed
    xboxDriver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
  }

  public void bindXboxOperator(int port) {
    var xboxOperator = new CommandXboxController(port);

    // intake
    xboxOperator
        .rightBumper()
        .whileTrue(Commands.startEnd(intake::intakeFuel, () -> {}, intake).withName("Intaking"));
  }

  public void configureAutoOptions() {
    autoSelector.addAuto(new AutoOption(Alliance.Blue, 1, new B_MoveForward1M(drive)));
    autoSelector.addAuto(new AutoOption(Alliance.Red, 1, new R_MoveStraight(drive)));
    autoSelector.addAuto(new AutoOption(Alliance.Blue, 2, new TraversingTheBump(drive)));
    autoSelector.addAuto(new AutoOption(Alliance.Red, 2, new R_MoveAndRotate(drive)));
    autoSelector.addAuto(new AutoOption(Alliance.Blue, 3, new B_Path(drive)));
  }

  public static Alliance getAlliance() {
    return allianceSelector.getAllianceColor();
  }
}
