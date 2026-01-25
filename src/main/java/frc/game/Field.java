package frc.game;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class Field {
  // Measured
  public static final Distance centerField_x_pos = Inches.of(325.06);
  public static final Distance centerField_y_pos = Inches.of(316.64);
  private static final Distance hub_x_centerPos = Inches.of(181.56);
  private static final Distance hub_x_len = Inches.of(47);
  private static final Distance hub_y_len = Inches.of(47);
  private static final Distance trench_y_len = Inches.of(49.86);
  private static final Distance bump_y_len = Inches.of(73);
  private static final Distance depot_y_centerPos = centerField_y_pos.plus(Inches.of(75.93));
  private static final Distance depot_x_len = Inches.of(27);
  private static final Distance depot_y_len = Inches.of(42);
  private static final Distance tower_y_centerPos = centerField_y_pos.minus(Inches.of(11.46));
  private static final Distance tower_x_len = Inches.of(43.8);
  private static final Distance tower_y_len = Inches.of(47);
  private static final Distance fuel_x_len = Inches.of(35.95 * 2);
  private static final Distance fuel_y_len = Inches.of(90.95 * 2);

  // Constructed
  private static final Distance field_x_len = centerField_x_pos.times(2);
  private static final Distance field_y_len = centerField_y_pos.times(2);
  private static final Distance allianceZone_x_len = hub_x_centerPos.minus(hub_x_len.div(2));
  private static final Distance neutralZone_x_len =
      centerField_x_pos.minus(hub_x_centerPos.plus(hub_x_len)).times(2);
  private static final Pose2d fieldCenter =
      new Pose2d(new Translation2d(centerField_x_pos, centerField_y_pos), Rotation2d.kZero);
  private static final Pose2d blueHubCenter =
      new Pose2d(new Translation2d(hub_x_centerPos, centerField_y_pos), Rotation2d.kZero);
  private static final Pose2d redHubCenter =
      new Pose2d(
          new Translation2d(centerField_x_pos.times(2).minus(hub_x_centerPos), centerField_y_pos),
          Rotation2d.kZero);

  enum Region {
    BlueZone(
        new Rectangle2d(
            new Translation2d(0, 0), new Translation2d(allianceZone_x_len, field_y_len))),
    NeutralZone(new Rectangle2d(fieldCenter, neutralZone_x_len, field_y_len)),
    RedZone(
        new Rectangle2d(
            new Translation2d(field_x_len.minus(allianceZone_x_len), Inches.of(0)),
            new Translation2d(field_x_len, field_y_len))),
    BlueHub(new Rectangle2d(blueHubCenter, hub_x_len, hub_y_len)),
    RedHub(new Rectangle2d(redHubCenter, hub_x_len, hub_y_len)),
    RedLeftTrench(
        new Rectangle2d(
            new Translation2d(
                field_x_len.minus(hub_x_centerPos).minus(hub_x_len.div(2)), Inches.of(0)),
            new Translation2d(
                field_x_len.minus(hub_x_centerPos).plus(hub_x_len.div(2)), trench_y_len))),
    RedRightTrench(
        new Rectangle2d(
            new Translation2d(
                field_x_len.minus(hub_x_centerPos).minus(hub_x_len.div(2)),
                field_y_len.minus(trench_y_len)),
            new Translation2d(
                field_x_len.minus(hub_x_centerPos).plus(hub_x_len.div(2)), field_y_len))),
    BlueLeftTrench(
        new Rectangle2d(
            new Translation2d(
                hub_x_centerPos.minus(hub_x_len.div(2)), field_y_len.minus(trench_y_len)),
            new Translation2d(hub_x_centerPos.plus(hub_x_len.div(2)), field_y_len))),
    BlueRightTrench(
        new Rectangle2d(
            new Translation2d(hub_x_centerPos.minus(hub_x_len.div(2)), Inches.of(0)),
            new Translation2d(hub_x_centerPos.plus(hub_x_len.div(2)), trench_y_len))),
    RedLeftBump(
        new Rectangle2d(
            new Translation2d(
                field_x_len.minus(hub_x_centerPos).minus(hub_x_len.div(2)),
                centerField_y_pos.minus(hub_y_len.div(2)).minus(bump_y_len)),
            new Translation2d(
                field_x_len.minus(hub_x_centerPos).plus(hub_x_len.div(2)),
                centerField_y_pos.minus(hub_y_len.div(2))))),
    RedRightBump(
        new Rectangle2d(
            new Translation2d(
                field_x_len.minus(hub_x_centerPos).minus(hub_x_len.div(2)),
                centerField_y_pos.plus(hub_y_len.div(2))),
            new Translation2d(
                field_x_len.minus(hub_x_centerPos).plus(hub_x_len.div(2)),
                centerField_y_pos.plus(hub_y_len.div(2)).plus(bump_y_len)))),
    BlueLeftBump(
        new Rectangle2d(
            new Translation2d(
                hub_x_centerPos.minus(hub_x_len.div(2)), centerField_y_pos.plus(hub_y_len.div(2))),
            new Translation2d(
                hub_x_centerPos.plus(hub_x_len.div(2)),
                centerField_y_pos.plus(hub_y_len.div(2)).plus(bump_y_len)))),
    BlueRightBump(
        new Rectangle2d(
            new Translation2d(
                hub_x_centerPos.minus(hub_x_len.div(2)),
                centerField_y_pos.minus(hub_y_len.div(2)).minus(bump_y_len)),
            new Translation2d(
                hub_x_centerPos.plus(hub_x_len.div(2)),
                centerField_y_pos.minus(hub_y_len.div(2))))),
    BlueDepot(
        new Rectangle2d(
            new Translation2d(Inches.of(0), depot_y_centerPos.minus(depot_y_len.div(2))),
            new Translation2d(depot_x_len, depot_y_centerPos.plus(depot_y_len.div(2))))),
    RedDepot(
        new Rectangle2d(
            new Translation2d(
                field_x_len.minus(depot_x_len),
                field_y_len.minus(depot_y_centerPos).minus(depot_y_len.div(2))),
            new Translation2d(
                field_x_len, field_y_len.minus(depot_y_centerPos).plus(depot_y_len.div(2))))),
    BlueTower(
        new Rectangle2d(
            new Translation2d(Inches.of(0), tower_y_centerPos.minus(tower_y_len.div(2))),
            new Translation2d(tower_x_len, tower_y_centerPos.plus(tower_y_len.div(2))))),
    RedTower(
        new Rectangle2d(
            new Translation2d(
                field_x_len.minus(tower_x_len),
                field_y_len.minus(tower_y_centerPos).minus(tower_y_len.div(2))),
            new Translation2d(
                field_x_len, field_y_len.minus(tower_y_centerPos).plus(tower_y_len.div(2))))),
    FuelArrangement(new Rectangle2d(fieldCenter, fuel_x_len, fuel_y_len)),
    Field(new Rectangle2d(new Translation2d(0, 0), new Translation2d(field_x_len, field_y_len)));

    private final Rectangle2d rect;

    private Region(Rectangle2d rect) {
      this.rect = rect;
    }

    public boolean contains(Pose2d pose) {
      return rect.contains(pose.getTranslation());
    }

    public Pose2d getCenter() {
      return rect.getCenter();
    }
  }

  public static Region getZone(Pose2d pose) {
    if (Region.BlueZone.contains(pose)) {
      return Region.BlueZone;
    }
    if (Region.RedZone.contains(pose)) {
      return Region.RedZone;
    }
    return Region.NeutralZone;
  }
}
