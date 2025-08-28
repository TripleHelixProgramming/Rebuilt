package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoMode;
import java.util.Optional;

public class AutoOption {
  private final Alliance allianceColor;
  private final int switchNumber;
  private final AutoMode autoMode;

  /**
   * Constructs a selectable autonomous mode option
   *
   * @param color Alliance for which the option is valid
   * @param option Selector switch index for which the option is valid
   * @param autoSupplier Supplies command which runs the autonomous mode
   */
  public AutoOption(Alliance color, int option, AutoMode autoMode) {
    this.allianceColor = color;
    this.switchNumber = option;
    this.autoMode = autoMode;
  }

  /**
   * Constructs a null autonomous mode option
   *
   * @param color Alliance for which the option is valid
   * @param option Selector switch index for which the option is valid
   */
  public AutoOption(Alliance color, int option) {
    this(color, option, null);
  }

  /**
   * @return Alliance for which the option is valid
   */
  public Alliance getAlliance() {
    return this.allianceColor;
  }

  /**
   * @return Color of the associated alliance
   */
  public Color getAllianceColor() {
    return Util.allianceToColor(getAlliance());
  }

  /**
   * @return Selector switch index for which the option is valid
   */
  public int getOptionNumber() {
    return this.switchNumber;
  }

  /**
   * @return The command which runs the selected autonomous mode
   */
  public synchronized Optional<Command> getAutoCommand() {
    return (autoMode == null) ? Optional.empty() : Optional.of(autoMode.getAutoRoutine().cmd());
  }

  public Optional<Pose2d> getInitialPose() {
    return (autoMode == null) ? Optional.empty() : autoMode.getInitialPose();
  }

  public synchronized String getName() {
    return (autoMode == null) ? "None; this slot reserved for no auto" : autoMode.getName();
  }
}
