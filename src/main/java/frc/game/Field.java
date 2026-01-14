package frc.game;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Field {
    public final static double FIELD_WIDTH = 317.7;
    public final static double ALLIANCE_ZONE_LENGTH = 158.6;
    public final static double NEUTRAL_ZONE_LENGTH = 283;

    enum Region {
        BlueZone(0,0, ALLIANCE_ZONE_LENGTH, FIELD_WIDTH), 
        NeutralZone(ALLIANCE_ZONE_LENGTH, 0, NEUTRAL_ZONE_LENGTH, FIELD_WIDTH), 
        RedZone(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH, 0, ALLIANCE_ZONE_LENGTH, FIELD_WIDTH), 
        RedLeftTrench(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH, FIELD_WIDTH, 65.65, 47),
        RedRightTrench(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH, 0, 65.65, 47), 
        BlueLeftTrench(ALLIANCE_ZONE_LENGTH, FIELD_WIDTH, 65.65, 47),
        BlueRightTrench(ALLIANCE_ZONE_LENGTH, 0, 65.65, 47), 
        BlueDepot(ALLIANCE_ZONE_LENGTH, 65.65, 42, 27), 
        RedDepot(FIELD_WIDTH, 91.4 + 73, 42, 27), 
        BlueTower( ALLIANCE_ZONE_LENGTH, 91.4, 49.25, 45), 
        RedTower( FIELD_WIDTH, 91.4, 49.25, 45), 
        BlueHub(ALLIANCE_ZONE_LENGTH, 91.4, 47, 47), 
        RedHub(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH, 91.4, 47, 47),
        RedLeftBump(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH, 135.8, 73, 44.4),
        RedRightBump(ALLIANCE_ZONE_LENGTH + NEUTRAL_ZONE_LENGTH, 47, 73, 44.4),
        BlueLeftBump(ALLIANCE_ZONE_LENGTH, 135.8, 73, 44.4),
        BlueRightBump(ALLIANCE_ZONE_LENGTH, 47, 73, 44.4),
        FuelArrangement(NEUTRAL_ZONE_LENGTH / 2 + 72 / 2, 317.7 - 47 * 2, 72, 206);

        private Region(double xpos, double ypos, double xlen, double ylen) {
            bottomRightCorner = new Pose2d(xpos, ypos, Rotation2d.kZero);
            this.xlen = xlen;
            this.ylen = ylen;
        }

        public final Pose2d bottomRightCorner;
        public final double xlen;
        public final double ylen;

        public boolean contains(Pose2d pose) {
            //TODO: implement this
            return false;
        }

    }

}
