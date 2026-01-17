package frc.lib;

import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.AutoLog;

public class AutoSelectorIO {

  public DigitalInput[] autoSelectionSwitches;

  public AutoSelectorIO(int[] ports) {
    this.autoSelectionSwitches = new DigitalInput[ports.length];
    for (int i = 0; i < ports.length; i++) {
      autoSelectionSwitches[i] = new DigitalInput(ports[i]);
    }
  }

  @AutoLog
  public static class AutoSelectorIOInputs {
    public int autoSwitchPosition = 0;
  }

  public void updateInputs(AutoSelectorIOInputs inputs) {
    inputs.autoSwitchPosition = getBinarySwitchPosition();
  }

  /**
   * @return The position of the autonomous selection switch
   */
  public int getRotarySwitchPosition() {
    for (int i = 0; i < autoSelectionSwitches.length; i++) {
      if (!autoSelectionSwitches[i].get()) {
        return i + 1;
      }
    }
    return 0; // failure of the physical switch
  }

  public int getBinarySwitchPosition() {
    int sum = 0;
    for (int i = 0; i < autoSelectionSwitches.length; i++) {
      if (!autoSelectionSwitches[i].get()) {
        sum += (1 << (i));
      }
    }
    return sum;
  }
}
