package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLog;

public class AllianceSelectorIO {

  public DigitalInput allianceSelectionSwitch;
  private Alliance currentColor;

  public AllianceSelectorIO(int port) {
    this.allianceSelectionSwitch = new DigitalInput(port);
  }

  @AutoLog
  public static class AllianceSelectorIOInputs {
    public Alliance allianceFromSwitch = Alliance.Red;
    public boolean agreementInAllianceInputs = false;
    public boolean allianceChanged = false;
  }

  public void updateInputs(AllianceSelectorIOInputs inputs) {
    inputs.allianceFromSwitch = allianceSelectionSwitch.get() ? Alliance.Red : Alliance.Blue;
    inputs.agreementInAllianceInputs = agreementInAllianceInputs(inputs);
    inputs.allianceChanged = changedAlliance(inputs);
  }

  private boolean changedAlliance(AllianceSelectorIOInputs inputs) {
    Alliance newColor = inputs.allianceFromSwitch;

    if (newColor.equals(currentColor)) return false;
    else {
      currentColor = newColor;
      return true;
    }
  }

  private boolean agreementInAllianceInputs(AllianceSelectorIOInputs inputs) {
    Optional<Alliance> allianceFromFMS = DriverStation.getAlliance();
    if (allianceFromFMS.isPresent()) {
      return inputs.allianceFromSwitch.equals(allianceFromFMS.get());
    } else return false;
  }
}
