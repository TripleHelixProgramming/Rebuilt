package frc.game;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GameState {
    enum GameZone {
        Neutral, Red, Blue;
    }
    public GameZone getCurrentZone(Pose2d pose) {
        if(Field.Region.BlueZone.contains(pose)) {
            return GameZone.Blue;
        }
        if(Field.Region.RedZone.contains(pose)) {
            return GameZone.Red;
        }
        if(Field.Region.NeutralZone.contains(pose)) {
            return GameZone.Neutral;
        }
        System.err.println("Impossible Field Location");
        return GameZone.Neutral;
    }
    public Alliance getActiveHub() {
        //TODO: Actually implement this
        return null;
    }   
    public Optional<Alliance> getMyAlliance() {
        return DriverStation.getAlliance();
    }

}
// import edu.wpi.first.wpilibj.DriverStation;
// String gameData;
// gameData = DriverStation.getGameSpecificMessage();
// if(gameData.length() > 0)
// {
//   switch (gameData.charAt(0))
//   {
//     case 'B' :
//       //Blue case code
//       break;
//     case 'R' :
//       //Red case code
//       break;
//     default :
//       //This is corrupt data
//       break;
//   }
// } else {
//   //Code for no data received yet
// }