package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.launcher.LauncherConstants.TurretConstants.*;
import static frc.robot.subsystems.launcher.LauncherConstants.ballRadius;
import static frc.robot.subsystems.launcher.LauncherConstants.ceilingHeight;
import static frc.robot.subsystems.launcher.LauncherConstants.gravity;
import static frc.robot.subsystems.launcher.LauncherConstants.impactAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.game.GameState;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final TurretIO turretIO;
  private final FlywheelIO flywheelIO;
  private final HoodIO hoodIO;

  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();
  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

  private final Supplier<Pose2d> chassisPoseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  private final Alert turretDisconnectedAlert;
  private final Alert flywheelDisconnectedAlert;
  private final Alert hoodDisconnectedAlert;

  private Translation3d vectorTurretBaseToHub = new Translation3d();
  private Pose3d turretBasePose = new Pose3d();
  private Translation3d v0_nominal = new Translation3d();

  // Ball simulation
  private final ArrayList<BallInFlight> balls = new ArrayList<>();
  private static final double kBallSpawnPeriod = 0.1; // seconds
  private double ballSpawnTimer = 0.0;

  public Launcher(
      Supplier<Pose2d> chassisPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      TurretIO turretIO,
      FlywheelIO flywheelIO,
      HoodIO hoodIO) {
    this.turretIO = turretIO;
    this.flywheelIO = flywheelIO;
    this.hoodIO = hoodIO;

    this.chassisPoseSupplier = chassisPoseSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    turretDisconnectedAlert = new Alert("Disconnected turret motor", AlertType.kError);
    flywheelDisconnectedAlert = new Alert("Disconnected flywheel motor", AlertType.kError);
    hoodDisconnectedAlert = new Alert("Disconnected hood motor", AlertType.kError);
  }

  @Override
  public void periodic() {
    turretIO.updateInputs(turretInputs);
    flywheelIO.updateInputs(flywheelInputs);
    hoodIO.updateInputs(hoodInputs);

    Logger.processInputs("Turret", turretInputs);
    Logger.processInputs("Flyweel", flywheelInputs);
    Logger.processInputs("Hood", hoodInputs);

    turretDisconnectedAlert.set(!turretInputs.connected);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);
    hoodDisconnectedAlert.set(!hoodInputs.connected);

    // Get vector from static target to turret
    var hubPose = GameState.getMyHubPose();
    turretBasePose = new Pose3d(chassisPoseSupplier.get()).plus(chassisToTurretBase);
    vectorTurretBaseToHub = hubPose.getTranslation().minus(turretBasePose.getTranslation());

    updateBalls();
  }

  public void stop() {
    turretIO.setOpenLoop(0.0);
    flywheelIO.setOpenLoop(0.0);
    hoodIO.setOpenLoop(0.0);
  }

  public void aimAtHub() {
    updateIntialVelocities(vectorTurretBaseToHub);

    ballSpawnTimer += Robot.defaultPeriodSecs;
    if (ballSpawnTimer >= kBallSpawnPeriod) {
      ballSpawnTimer = 0.0;

      balls.add(
          new BallInFlight(
              new Translation3d(
                  turretBasePose.getX(), turretBasePose.getY(), turretBasePose.getZ()),
              new Translation3d(v0_nominal.getX(), v0_nominal.getY(), v0_nominal.getZ())));
    }

    flywheelIO.setVelocity(v0_nominal.getNorm());

    // Get turret linear velocities (m/s)
    var robotRelative = chassisSpeedsSupplier.get();
    var fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelative, turretBasePose.toPose2d().getRotation());
    Translation3d turretBaseSpeeds = getTurretBaseSpeeds(fieldRelative);

    var residualVelocities = v0_nominal.minus(turretBaseSpeeds);
    Rotation2d turretSetpoint =
        new Rotation2d(residualVelocities.getX(), residualVelocities.getY());

    // // Create unit vectors
    // var radialUnit = vectorHubToTurretBase.div(vectorHubToTurretBase.getNorm());
    // var tangentialUnit = radialUnit.rotateBy(Rotation2d.kCCW_90deg);

    // // Compute dot products
    // var radialMagnitude = turretBaseSpeeds.dot(radialUnit);
    // var tangentialMagnitude = turretBaseSpeeds.dot(tangentialUnit);

    // // Create radial and tangential components of turret velocity (m/s) relative to hub
    // var radialVelocity = radialUnit.times(radialMagnitude);
    // var tangentialVelocity = tangentialUnit.times(tangentialMagnitude);

    // // Calculate distance to lag the target
    // var lag = tangentialVelocity.times(fuelTravelTime.in(Seconds));
    // var dynamicTarget = hubPose.minus(lag);
    // target = new Pose2d(dynamicTarget, Rotation2d.kZero);

    // // Get vector from dynamic target to turret
    // dynamicTargetToTurretBase = turretBasePose.getTranslation().minus(dynamicTarget);

    // // Get angular velocity omega, where V = omega x R
    // var apparentAngularVelocityRadPerSec =
    //     turretBaseSpeeds.cross(dynamicTargetToTurretBase)
    //         / dynamicTargetToTurretBase.getSquaredNorm();

    // turretOrientationSetpoint =
    //     dynamicTargetToTurretBase.unaryMinus().getAngle().minus(turretBasePose.getRotation());

    // double feedforwardVolts =
    //     RobotConstants.kNominalVoltage
    //         * -(2 * robotRelative.omegaRadiansPerSecond // TODO: Why is this factor 2 needed?
    //             + apparentAngularVelocityRadPerSec)
    //         / turnMaxAngularVelocity.in(RadiansPerSecond);

    // turretIO.setPosition(turretOrientationSetpoint, feedforwardVolts);
    turretIO.setPosition(
        turretSetpoint.minus(turretBasePose.toPose2d().getRotation()),
        RadiansPerSecond.of(robotRelative.omegaRadiansPerSecond).unaryMinus());
  }

  @AutoLogOutput(key = "Launcher/TurretPose")
  public Pose2d getTurretPose() {
    return turretBasePose.toPose2d().plus(new Transform2d(0, 0, turretInputs.position));
  }

  // @AutoLogOutput(key = "Turret/IsOnTarget")
  // public boolean isOnTarget() {
  //   return turretInputs.position.minus(turretOrientationSetpoint).getMeasure().abs(Radians)
  //           * dynamicTargetToTurretBase.getNorm()
  //       < (hubWidth.in(Meters) / 2.0);
  // }

  private Translation3d getTurretBaseSpeeds(ChassisSpeeds chassisSpeeds) {
    double vx = chassisSpeeds.vxMetersPerSecond;
    double vy = chassisSpeeds.vyMetersPerSecond;
    double omega = chassisSpeeds.omegaRadiansPerSecond;
    var baseSpeeds =
        new Translation3d(
            vx - omega * chassisToTurretBase.getY(), vy + omega * chassisToTurretBase.getX(), 0);
    Logger.recordOutput("Launcher/BaseSpeeds", baseSpeeds);
    return baseSpeeds;
  }

  private void updateIntialVelocities(Translation3d distances) {
    double dr = Math.hypot(distances.getX(), distances.getY());
    double dz = distances.getZ();

    double v_0r = dr * Math.sqrt(gravity / (2 * (dz + dr * impactAngle.getTan())));
    double v_0z = (gravity * dr) / v_0r - v_0r * impactAngle.getTan();

    double v_0x = v_0r * distances.toTranslation2d().getAngle().getCos();
    double v_0y = v_0r * distances.toTranslation2d().getAngle().getSin();

    v0_nominal = new Translation3d(v_0x, v_0y, v_0z);
    Logger.recordOutput("Launcher/InitialVelocities", v0_nominal);
    Logger.recordOutput("Launcher/InitialSpeedMetersPerSecond", v0_nominal.getNorm());
    Logger.recordOutput(
        "Launcher/VerticalLaunchAngleDegrees",
        new Translation2d(v_0r, v_0z).getAngle().getDegrees());
    Logger.recordOutput(
        "Launcher/HorizontalLaunchAngleDegrees",
        v0_nominal.toTranslation2d().getAngle().getDegrees());
    Logger.recordOutput("Launcher/NominalTravelTime", dr / v_0r);

    var max_height = turretBasePose.getZ() + v_0z * v_0z / (2 * gravity);
    Logger.recordOutput("Launcher/NominalMaxHeight", max_height);

    boolean clearsCeiling = Meters.of(max_height).plus(ballRadius).lt(ceilingHeight);
    Logger.recordOutput("Launcher/ClearsCeiling", clearsCeiling);
  }

  private static class BallInFlight {
    Translation3d position;
    Translation3d velocity;

    BallInFlight(Translation3d position, Translation3d velocity) {
      this.position = position;
      this.velocity = velocity;
    }
  }

  private void updateBalls() {
    double dt = Robot.defaultPeriodSecs;
    double hubZ = GameState.getMyHubPose().getZ();

    balls.removeIf(
        ball -> {
          // Integrate velocity
          ball.velocity = ball.velocity.plus(new Translation3d(0, 0, -gravity * dt));

          // Integrate position
          ball.position = ball.position.plus(ball.velocity.times(dt));

          // Remove when below target height and falling
          return ball.position.getZ() < hubZ && ball.velocity.getZ() < 0;
        });
  }

  @AutoLogOutput(key = "Launcher/BallTrajectory")
  public Translation3d[] getBallTrajectory() {
    Translation3d[] t = new Translation3d[balls.size()];
    for (int i = 0; i < balls.size(); i++) {
      t[i] = balls.get(i).position;
    }
    return t;
  }
}
