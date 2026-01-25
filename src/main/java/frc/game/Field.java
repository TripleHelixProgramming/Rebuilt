package frc.game;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class Field {
  public static final Distance centerField_x_pos = Inches.of(325.06);
  public static final Distance centerField_y_pos = Inches.of(316.64);
  private static final Distance hub_x_pos = Inches.of(181.56);
  private static final Distance hub_x_len = Inches.of(47);
  private static final Distance hub_y_len = Inches.of(47);
  private static final Distance trench_y_len = Inches.of(49.86);

  private static final Distance field_x_len = centerField_x_pos.times(2);
  private static final Distance field_y_len = centerField_y_pos.times(2);
  private static final Distance allianceZone_x_len = hub_x_pos.minus(hub_x_len.div(2));
  private static final Distance neutralZone_x_len = centerField_x_pos.minus(hub_x_pos.plus(hub_x_len)).times(2);
  private static final Pose2d fieldCenter = new Pose2d(new Translation2d(centerField_x_pos, centerField_y_pos), Rotation2d.kZero);
  private static final Pose2d blueHubCenter = new Pose2d(new Translation2d(hub_x_pos, centerField_y_pos), Rotation2d.kZero);
  private static final Pose2d redHubCenter = new Pose2d(new Translation2d(centerField_x_pos.times(2).minus(hub_x_pos), centerField_y_pos), Rotation2d.kZero);

  enum Region {
    BlueZone(new Rectangle2d(new Translation2d(0, 0), new Translation2d(allianceZone_x_len, field_y_len))),
    NeutralZone(new Rectangle2d(fieldCenter, neutralZone_x_len, field_y_len)),
    RedZone(new Rectangle2d(new Translation2d(field_x_len.minus(allianceZone_x_len), Inches.of(0)), new Translation2d(field_x_len, field_y_len))),
    BlueHub(new Rectangle2d(blueHubCenter, hub_x_len, hub_y_len)),
    RedHub(new Rectangle2d(redHubCenter, hub_x_len, hub_y_len)),
    RedLeftTrench(new Rectangle2d(new Translation2d(field_x_len.minus(hub_x_pos).minus(hub_x_len.div(2)), Inches.of(0)), new Translation2d(field_x_len.minus(hub_x_pos).plus(hub_x_len.div(2)), trench_y_len))),
    RedRightTrench(new Rectangle2d(new Translation2d(field_x_len.minus(hub_x_pos).minus(hub_x_len.div(2)), field_y_len.minus(trench_y_len)), new Translation2d(field_x_len.minus(hub_x_pos).plus(hub_x_len.div(2)), field_y_len))),
    BlueLeftTrench(new Rectangle2d(new Translation2d(hub_x_pos.minus(hub_x_len.div(2)), field_y_len.minus(trench_y_len)), new Translation2d(hub_x_pos.plus(hub_x_len.div(2)), field_y_len))),
    BlueRightTrench(new Rectangle2d(new Translation2d(hub_x_pos.minus(hub_x_len.div(2)), Inches.of(0)), new Translation2d(hub_x_pos.plus(hub_x_len.div(2)), trench_y_len))),
    BlueDepot(build().xpos(0).ypos(centerField_y_pos + 75.93 - 42/2).xlen(27).ylen(42)),
    RedDepot(build().xpos(2*centerField_x_pos - 27).ypos(centerField_y_pos - 75.93 - 42/2).xlen(27).ylen(42)),
    // BlueTower(build().xpos(ALLIANCE_ZONE_LENGTH).ypos(91.4).xlen(49.25).ylen(45)),
    // RedTower(build().xpos(FIELD_WIDTH).ypos(91.4).xlen(49.25).ylen(45)),
    RedLeftBump(
        build().xpos(allianceZone_x_len + 47 + neutralZone_x_len).ypos(49.86 + 12 + 73 + 47).xlen(47).ylen(73)),
    RedRightBump(
        build().xpos(allianceZone_x_len + 47 + neutralZone_x_len).ypos(49.86 + 12).xlen(47).ylen(73)),
    BlueLeftBump(build().xpos(allianceZone_x_len).ypos(49.86 + 12 + 73 + 47).xlen(47).ylen(73)),
    BlueRightBump(build().xpos(allianceZone_x_len).ypos(49.86 + 12).xlen(47).ylen(73)),
    FuelArrangement(new Rectangle2d(fieldCenter, Inches.of(35.95*2), Inches.of(90.95*2)));

    public final Rectangle2d rect;

    private Region(Rectangle2d rect) {
      this.rect = rect;
    }

    public boolean contains(Pose2d pose) {
      return rect.contains(pose.getTranslation());
    }
  }

  public static Region getZone(Pose2d pose) {
    if (pose.getX() < Region.NeutralZone.xpos) {
      return Region.BlueZone;
    }
    if (pose.getX() < Region.RedZone.xpos) {
      return Region.NeutralZone;
    }
    return Region.RedZone;
  }
}
