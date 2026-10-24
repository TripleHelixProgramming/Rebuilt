package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
 * Runs the real Drive with fake modules that report per-sample validity, as ModuleIOTalonFX does.
 * The dead module's connection flags still read true, as they do for the half second before the
 * debounce catches up, so these pass only if Module uses the per-sample validity.
 */
class DriveOdometryValidityTest {
  private static double distanceMeters;
  private static boolean gyroConnected;

  /** A module that reports one sample per loop; module 1 is dead and its samples are invalid. */
  private static final class FakeModule implements ModuleIO {
    private final boolean dead;

    FakeModule(int index) {
      dead = index == 1;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      double positionRad = dead ? 0.0 : distanceMeters / DriveConstants.WHEEL_RADIUS_METERS;
      inputs.driveConnected = true;
      inputs.turnConnected = true;
      inputs.turnEncoderConnected = true;
      inputs.drivePositionRad = positionRad;
      inputs.turnPosition = Rotation2d.kZero;
      inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
      inputs.odometryDrivePositionsRad = new double[] {positionRad};
      inputs.odometryTurnPositions = new Rotation2d[] {Rotation2d.kZero};
      inputs.odometryDriveValid = new boolean[] {!dead};
      inputs.odometryTurnValid = new boolean[] {!dead};
    }
  }

  private static final class FakeGyro implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected = gyroConnected;
      inputs.odometryYawTimestamps =
          gyroConnected ? new double[] {Timer.getFPGATimestamp()} : new double[] {};
      inputs.odometryYawPositions =
          gyroConnected ? new Rotation2d[] {Rotation2d.kZero} : new Rotation2d[] {};
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
  void reset() {
    distanceMeters = 0.0;
    gyroConnected = true;
  }

  /** Drives straight at 1 m/s for 3 s and returns the drive. */
  private static Drive driveThreeMeters() {
    var drive =
        new Drive(
            new FakeGyro(),
            new FakeModule(0),
            new FakeModule(1),
            new FakeModule(2),
            new FakeModule(3));
    drive.setPose(new Pose2d(2.0, 2.0, Rotation2d.kZero));
    for (int i = 0; i < 150; i++) {
      distanceMeters += 0.02;
      SimHooks.stepTiming(0.02);
      drive.periodic();
    }
    return drive;
  }

  @Test
  @DisplayName("A module whose samples are invalid is filled in from the others")
  void invalidModuleIsFilledIn() {
    var pose = driveThreeMeters().getPose();
    assertEquals(5.0, pose.getX(), 0.02);
    assertEquals(2.0, pose.getY(), 0.02);
  }

  @Test
  @DisplayName("Without a gyro, a module whose samples are invalid adds no false rotation")
  void invalidModuleWithoutGyro() {
    gyroConnected = false;
    var pose = driveThreeMeters().getPose();
    assertEquals(0.0, pose.getRotation().getRadians(), 1e-6);
    assertEquals(5.0, pose.getX(), 0.02);
    assertEquals(2.0, pose.getY(), 0.02);
  }
}
