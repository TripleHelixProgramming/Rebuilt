package frc.game;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Field {
  public static final double FIELD_WIDTH = 317.7;
  public static final double ALLIANCE_ZONE_LENGTH = 158.6;
  public static final double NEUTRAL_ZONE_LENGTH = 283;

  enum Region {
    BlueZone(build().xpos(0).ypos(0).xlen(ALLIANCE_ZONE_LENGTH).ylen(FIELD_WIDTH)),
    NeutralZone(
        build().xpos(ALLIANCE_ZONE_LENGTH).ypos(0).xlen(NEUTRAL_ZONE_LENGTH).ylen(FIELD_WIDTH)),
    RedZone(
        build()
            .xpos(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH)
            .ypos(0)
            .xlen(ALLIANCE_ZONE_LENGTH)
            .ylen(FIELD_WIDTH)),
    RedLeftTrench(
        build()
            .xpos(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH)
            .ypos(FIELD_WIDTH)
            .xlen(65.65)
            .ylen(47)),
    RedRightTrench(
        build().xpos(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH).ypos(0).xlen(65.65).ylen(47)),
    BlueLeftTrench(build().xpos(ALLIANCE_ZONE_LENGTH).ypos(FIELD_WIDTH).xlen(65.65).ylen(47)),
    BlueRightTrench(build().xpos(ALLIANCE_ZONE_LENGTH).ypos(0).xlen(65.65).ylen(47)),
    BlueDepot(build().xpos(ALLIANCE_ZONE_LENGTH).ypos(65.65).xlen(42).ylen(27)),
    RedDepot(build().xpos(FIELD_WIDTH).ypos(91.4 + 73).xlen(42).ylen(27)),
    BlueTower(build().xpos(ALLIANCE_ZONE_LENGTH).ypos(91.4).xlen(49.25).ylen(45)),
    RedTower(build().xpos(FIELD_WIDTH).ypos(91.4).xlen(49.25).ylen(45)),
    BlueHub(build().xpos(ALLIANCE_ZONE_LENGTH).ypos(138.65).xlen(47).ylen(47)),
    RedHub(build().xpos(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH).ypos(138.65).xlen(47).ylen(47)),
    RedLeftBump(
        build().xpos(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH).ypos(135.8).xlen(73).ylen(44.4)),
    RedRightBump(
        build().xpos(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH).ypos(47).xlen(73).ylen(44.4)),
    BlueLeftBump(build().xpos(ALLIANCE_ZONE_LENGTH).ypos(135.8).xlen(73).ylen(44.4)),
    BlueRightBump(build().xpos(ALLIANCE_ZONE_LENGTH).ypos(47).xlen(73).ylen(44.4)),
    FuelArrangement(
        build().xpos(NEUTRAL_ZONE_LENGTH / 2 + 72 / 2).ypos(317.7 - 47 * 2).xlen(72).ylen(206));

    public final double xpos;
    public final double ypos;
    public final double xlen;
    public final double ylen;
    public final Translation2d center;

    private Region(Builder builder) {
      this.xpos = builder.xpos;
      this.ypos = builder.ypos;
      this.xlen = builder.xlen;
      this.ylen = builder.ylen;
      this.center = new Translation2d(Inches.of(xpos + xlen / 2), Inches.of(ypos + ylen / 2));
    }

    public static Builder build() {
      return new Builder();
    }

    public static class Builder {
      double xpos;
      double ypos;
      double xlen;
      double ylen;

      public Builder xpos(double xpos) {
        this.xpos = xpos;
        return this;
      }

      public Builder ypos(double ypos) {
        this.ypos = ypos;
        return this;
      }

      public Builder xlen(double xlen) {
        this.xlen = xlen;
        return this;
      }

      public Builder ylen(double ylen) {
        this.ylen = ylen;
        return this;
      }
    }

    public boolean contains(Pose2d pose) {
      return (pose.getX() >= xpos)
          && (pose.getX() < xpos + xlen)
          && (pose.getY() >= ypos)
          && (pose.getY() < ypos + ylen);
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
