package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Replays the two drive failures from VACHE through the real Drive, with fake hardware reporting
 * the way the logs show, and checks that the pose neither jumps nor stops taking vision.
 *
 * <p>The fake modules report like logs recorded before per-sample validity existed: positions,
 * timestamps, and connection flags only.
 */
class DriveOdometryDropoutTest {
  private static final Matrix<N3, N1> VISION_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.5);

  /** What the fake hardware reports. Shared by the gyro and the four modules. */
  private static final class Hardware {
    double distanceMeters = 0.0; // Every module's wheel travel; the robot drives straight
    boolean connected = true;
    double frozenTimestamp = Double.NaN; // When set, every sample carries this timestamp
    double timestampLag = 0.0; // Subtracted from every sample's timestamp
    boolean deadModule1 = false; // Module 1 reports 0 and disconnected
    boolean gyroConnected = true;

    double timestamp() {
      return Double.isNaN(frozenTimestamp)
          ? Timer.getFPGATimestamp() - timestampLag
          : frozenTimestamp;
    }
  }

  private static Hardware hardware;

  private static final class ScriptedModule implements ModuleIO {
    private final int index;

    ScriptedModule(int index) {
      this.index = index;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      boolean dead = hardware.deadModule1 && index == 1;
      double positionRad =
          dead ? 0.0 : hardware.distanceMeters / DriveConstants.WHEEL_RADIUS_METERS;
      inputs.driveConnected = hardware.connected && !dead;
      inputs.turnConnected = hardware.connected && !dead;
      inputs.turnEncoderConnected = true;
      inputs.drivePositionRad = positionRad;
      inputs.turnPosition = Rotation2d.kZero;
      inputs.odometryTimestamps = new double[] {hardware.timestamp()};
      inputs.odometryDrivePositionsRad = new double[] {positionRad};
      inputs.odometryTurnPositions = new Rotation2d[] {Rotation2d.kZero};
    }
  }

  private static final class ScriptedGyro implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = hardware.gyroConnected;
      inputs.yawPosition = Rotation2d.kZero;
      inputs.odometryYawTimestamps =
          hardware.gyroConnected ? new double[] {hardware.timestamp()} : new double[] {};
      inputs.odometryYawPositions =
          hardware.gyroConnected ? new Rotation2d[] {Rotation2d.kZero} : new Rotation2d[] {};
    }
  }

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  @AfterAll
  static void resumeTiming() {
    SimHooks.resumeTiming();
  }

  @BeforeEach
  void resetHardware() {
    hardware = new Hardware();
  }

  private static Drive newDrive() {
    return new Drive(
        new ScriptedGyro(),
        new ScriptedModule(0),
        new ScriptedModule(1),
        new ScriptedModule(2),
        new ScriptedModule(3));
  }

  private static void loop(Drive drive) {
    SimHooks.stepTiming(0.02);
    drive.periodic();
  }

  private static double error(Drive drive, Pose2d truth) {
    return drive.getPose().getTranslation().getDistance(truth.getTranslation());
  }

  @Test
  @DisplayName("A CAN dropout neither moves the pose nor stops vision correcting it (VACHE Q27)")
  void canDropout() {
    var drive = newDrive();
    drive.setPose(new Pose2d(2.0, 2.0, Rotation2d.kZero));
    for (int i = 0; i < 100; i++) {
      hardware.distanceMeters += 0.02; // 1 m/s for 2 s
      loop(drive);
    }
    var truth = new Pose2d(4.0, 2.0, Rotation2d.kZero);
    assertEquals(0.0, error(drive, truth), 0.01, "Odometry tracks the drive");

    // The CANivore leaves USB: positions read 0 and timestamps freeze minutes in the past. The
    // debounced connection flags drop 0.1 s later. The robot can't move.
    hardware.frozenTimestamp = Timer.getFPGATimestamp() - 585.0;
    hardware.distanceMeters = 0.0;
    for (int i = 0; i < 5; i++) {
      loop(drive);
    }
    hardware.connected = false;
    for (int i = 0; i < 50; i++) {
      loop(drive);
    }
    assertEquals(0.0, error(drive, truth), 0.05, "The reset to zero is not motion");

    // A pose reset sticks: the next samples don't move it
    var reset = new Pose2d(5.0, 3.0, Rotation2d.kZero);
    drive.setPose(reset);
    loop(drive);
    assertEquals(0.0, error(drive, reset), 0.01, "The reset pose holds");

    // Vision must still correct the pose: start 1.4 m off and see the true pose at 10 Hz for 3 s
    for (int i = 0; i < 150; i++) {
      if (i % 5 == 0) {
        drive.addVisionMeasurement(truth, Timer.getTimestamp() - 0.03, VISION_STD_DEVS);
      }
      loop(drive);
    }
    assertEquals(0.0, error(drive, truth), 0.1, "Vision corrections accumulate");
  }

  @Test
  @DisplayName("A dead module and lagging timestamps don't stop the pose tracking (VACHE Q32)")
  void deadModuleWithLaggingTimestamps() {
    var drive = newDrive();
    hardware.deadModule1 = true; // Dead all match
    hardware.timestampLag = 60.0; // One stale signal of eight drags timestamps back
    drive.setPose(new Pose2d(2.0, 2.0, Rotation2d.kZero));
    double worst = 0.0;
    for (int i = 0; i < 250; i++) {
      hardware.distanceMeters += 0.02; // 1 m/s for 5 s
      hardware.timestampLag += 0.02 / 8.0;
      var truth = new Pose2d(2.0 + hardware.distanceMeters, 2.0, Rotation2d.kZero);
      if (i % 5 == 0) {
        drive.addVisionMeasurement(truth, Timer.getTimestamp(), VISION_STD_DEVS);
      }
      loop(drive);
      worst = Math.max(worst, error(drive, truth));
    }
    assertTrue(worst < 0.15, "Pose stays within 0.15 m of the robot; worst was " + worst);

    // Vision must still correct the pose: start 1.4 m off and see the true pose at 10 Hz for 3 s
    var truth = new Pose2d(2.0 + hardware.distanceMeters, 2.0, Rotation2d.kZero);
    drive.setPose(truth.plus(new Transform2d(1.0, 1.0, Rotation2d.kZero)));
    for (int i = 0; i < 150; i++) {
      hardware.timestampLag += 0.02 / 8.0;
      if (i % 5 == 0) {
        drive.addVisionMeasurement(truth, Timer.getTimestamp(), VISION_STD_DEVS);
      }
      loop(drive);
    }
    assertEquals(0.0, error(drive, truth), 0.1, "Vision corrections accumulate");
  }

  @Test
  @DisplayName("Without a gyro, a dead module adds no false rotation")
  void deadModuleWithoutGyro() {
    hardware.gyroConnected = false;
    hardware.deadModule1 = true;
    var drive = newDrive();
    drive.setPose(new Pose2d(2.0, 2.0, Rotation2d.kZero));
    for (int i = 0; i < 150; i++) {
      hardware.distanceMeters += 0.02; // 1 m/s for 3 s
      loop(drive);
    }
    var truth = new Pose2d(2.0 + hardware.distanceMeters, 2.0, Rotation2d.kZero);
    assertEquals(0.0, drive.getPose().getRotation().getRadians(), 1e-6, "Heading holds");
    assertEquals(0.0, error(drive, truth), 0.02, "Odometry tracks the drive");
  }
}
