package frc.lib;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.OIConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * This class will create joysticks for all 6 slots and allow you to retrieve them based on their
 * display name.
 */
public class ControllerPatroller {

  public static final class Constants {
    public static final String kXbox = "XBOX";
    public static final String kPS4 = "P";
    public static final String kRadioMaster = "TX16S";
    public static final String kZorro = "Zorro";

    public static final String[] kDriverControllerNames = new String[] {kZorro};
    public static final String[] kOperatorControllerNames = new String[] {kXbox};

    public static final class DpadDirection {
      public static int kUp = 0;
      public static int kRight = 90;
      public static int kDown = 180;
      public static int kLeft = 270;
    }
  }

  private static ControllerPatroller patroller = new ControllerPatroller();

  /**
   * Retrieve the singleton instance of the ControllerPatroller
   *
   * @return the instance of the ControllerPatroller
   */
  public static ControllerPatroller getInstance() {
    if (null == patroller) {
      patroller = new ControllerPatroller();
    }
    return patroller;
  }

  private List<GenericHID> controllers = new ArrayList<>();
  private List<String> controllerNames = new ArrayList<>();

  private ControllerPatroller() {
    // Create a joystick at each port so we can check their names later
    // Most of these objects will go unused
    for (int i = 0; i < 6; i++) {
      controllers.add(new GenericHID(i));
      controllerNames.add(controllers.get(i).getName());
    }
  }

  /**
   * Detect if the list of controllers connected to USB has changed.
   *
   * <p>If so, update the list of controllers and return TRUE.
   *
   * @return has the list of controllers changed?
   */
  public boolean controllersChanged() {

    boolean changed = false;
    for (int i = 0; i < 6; i++) {
      String controllerName = controllers.get(i).getName();
      if (!controllerNames.get(i).equals(controllerName)) {
        changed = true;
        controllerNames.set(i, controllerName);
      }
    }
    return changed;
  }

  /**
   * @param name string to look for in list of joystick names
   * @return Joystick or null
   */
  public Optional<GenericHID> find(String name) {
    // Filter the list of controllers for ones that contain the provided name
    Optional<GenericHID> joystick =
        controllers.stream()
            .filter(c -> c.getName().toLowerCase().contains(name.toLowerCase()))
            .findFirst();

    return joystick;
  }

  /**
   * @param name the name of the controller to access
   * @param defaultPort the port to access if a controller with the provided name is not found
   * @return the joystick with the provided name or at the provided defaultPort
   */
  public GenericHID get(String name, int defaultPort) {
    // Filter the list of controllers for ones that contain the provided name
    Optional<GenericHID> joystick = find(name);

    if (joystick.isPresent()) {
      return joystick.get();
    }

    // If we didn't find a controller with the provided name return the one at the
    // default port
    return controllers.get(defaultPort);
  }

  public GenericHID get(String[] names, int defaultPort) {
    // Filter the list of controllers for ones that contain the provided name
    for (String name : names) {
      Optional<GenericHID> joystick = find(name);
      if (joystick.isPresent()) {
        return joystick.get();
      }
    }
    // If we didn't find a controller with the provided name return the one at the
    // default port
    return controllers.get(defaultPort);
  }

  public int findDriverPort() {
    GenericHID driver =
        get(Constants.kDriverControllerNames, OIConstants.kDefaultDriverControllerPort);
    return driver.getPort();
  }

  public int findOperatorPort() {
    GenericHID op =
        get(Constants.kOperatorControllerNames, OIConstants.kDefaultOperatorControllerPort);
    return op.getPort();
  }
}
