package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.Objects;
import java.util.Set;
import java.util.function.IntConsumer;
import org.littletonrobotics.junction.Logger;

/**
 * Manages the selection and binding of controllers for driver and operator roles as a singleton.
 *
 * <p>This class scans for connected controllers and matches them against a predefined,
 * priority-ordered list of configurations. It automatically handles controller connections and
 * disconnections, rebinding controls as needed. The {@code scan()} method must be called
 * periodically to check for controller changes.
 *
 * <p><b>Priority:</b> The selection of controllers is prioritized in the order that configurations
 * are provided to the {@code configure()} method. The first matching configuration found for each
 * role (DRIVER, then OPERATOR) will be used. The operator search will not consider the port already
 * assigned to the driver.
 *
 * <p><b>Usage:</b>
 *
 * <p>1. Call the static {@code configure()} method a single time in {@code robotInit()} to set up
 * the desired controller configurations.
 *
 * <p>2. Periodically call {@code ControllerSelector.getInstance().scan()} from a loop in your robot
 * code (e.g., using a Notifier or from {@code disabledPeriodic()}) to handle controller changes.
 */
public class ControllerSelector {

  private static ControllerSelector instance;

  /**
   * Configures the ControllerSelector singleton with the specified controller configurations. This
   * method must be called once before getting the instance.
   *
   * @param configs A varargs list of controller configurations.
   */
  public static void configure(ControllerConfig... configs) {
    if (instance == null) {
      instance = new ControllerSelector(configs);
    }
  }

  /**
   * Returns the singleton instance of the ControllerSelector.
   *
   * @throws IllegalStateException if configure() has not been called yet.
   * @return The singleton instance.
   */
  public static ControllerSelector getInstance() {
    if (instance == null) {
      throw new IllegalStateException(
          "ControllerSelector has not been configured. Call configure() first.");
    }
    return instance;
  }

  /** Defines the possible functions for a controller: DRIVER or OPERATOR. */
  public enum ControllerFunction {
    DRIVER,
    OPERATOR
  }

  /**
   * Defines the supported controller hardware types and the string to identify them by in the
   * driver station.
   */
  public enum ControllerType {
    ZORRO("Zorro"),
    XBOX("XBOX"),
    RADIOMASTER("TX16S"),
    PS4("P");

    private final String deviceName;

    ControllerType(String deviceName) {
      this.deviceName = deviceName;
    }

    /** Returns the identifying string for the controller in the driver station. */
    String getDeviceName() {
      return deviceName;
    }
  }

  /**
   * A data class that encapsulates the configuration for a controller binding. It includes the
   * modes in which the configuration is valid, the controller function (DRIVER or OPERATOR), the
   * controller type, and a callback function to bind the controller's commands.
   */
  public static class ControllerConfig {
    public final Set<Mode> modes;
    public final ControllerFunction controllerFunction;
    public final ControllerType controllerType;
    public final IntConsumer bindingCallback;

    /**
     * Constructs a new ControllerConfig object.
     *
     * @param controllerFunction The function of the controller (DRIVER or OPERATOR).
     * @param controllerType The type of the controller (e.g., XBOX, ZORRO).
     * @param bindingCallback The callback function that binds the controller's commands. This
     *     function takes the port number of the controller as an argument.
     * @param modes The modes in which this configuration is valid (e.g., REAL, SIM).
     */
    public ControllerConfig(
        ControllerFunction controllerFunction,
        ControllerType controllerType,
        IntConsumer bindingCallback,
        Mode... modes) {
      this.modes = Set.of(modes);
      this.controllerFunction = controllerFunction;
      this.controllerType = controllerType;
      this.bindingCallback = bindingCallback;
    }
  }

  private static final int NUM_CONTROLLER_PORTS = DriverStation.kJoystickPorts;

  private final ControllerConfig[] controllerConfigs;
  private final GenericHID[] controllers;
  private final String[] controllerNames;

  /**
   * Constructs a new ControllerSelector object. This is private to enforce the singleton pattern.
   *
   * @param configs A varargs list of controller configurations.
   */
  private ControllerSelector(ControllerConfig... configs) {
    this.controllerConfigs = configs;

    // Create a GenericHID object for each port to allow polling for names.
    controllers = new GenericHID[NUM_CONTROLLER_PORTS];
    controllerNames = new String[NUM_CONTROLLER_PORTS];
    for (int i = 0; i < NUM_CONTROLLER_PORTS; i++) {
      controllers[i] = new GenericHID(i);
    }

    // Perform the initial scan and binding.
    scan();
  }

  /**
   * Compares the currently connected controllers to a cached list to detect changes. If a change is
   * found, the cache is updated. This method avoids allocating new objects.
   *
   * @return True if the list of connected controllers has changed, false otherwise.
   */
  private boolean controllersChanged() {
    boolean changed = false;
    for (int i = 0; i < NUM_CONTROLLER_PORTS; i++) {
      var currentName = controllers[i].getName();
      // Use Objects.equals for null-safe comparison.
      // This handles the initial case where the cached name is an empty string.
      if (!Objects.equals(currentName, controllerNames[i])) {
        controllerNames[i] = currentName; // Update the cache
        changed = true;
      }
    }
    return changed;
  }

  /**
   * Checks if a controller name matches the expected controller type, handling nulls and
   * case-insensitivity.
   *
   * @param controllerName The name of the controller to check.
   * @param controllerType The expected type of the controller.
   * @return True if the controller name matches the controller type, false otherwise.
   */
  private boolean controllerNameMatchesType(String controllerName, ControllerType controllerType) {
    if (controllerName == null) {
      return false;
    }
    return controllerName.toLowerCase().contains(controllerType.getDeviceName().toLowerCase());
  }

  /**
   * Scans for controller changes and re-binds controls if necessary. This method should be called
   * periodically (e.g., in disabledPeriodic or a dedicated Notifier).
   *
   * <p>The process is as follows:
   *
   * <p>1. Checks if the connected controllers have changed. If not, it returns immediately.
   *
   * <p>2. Clears all existing command bindings to ensure a clean state.
   *
   * <p>3. Iterates through the configurations to find the highest-priority match for the DRIVER
   * controller. If found, its binding callback is executed immediately.
   *
   * <p>4. Iterates again to find the highest-priority match for the OPERATOR controller, ensuring
   * it doesn't use the same port as the driver. If found, its binding callback is executed
   * immediately.
   *
   * <p>5. Logs the final controller assignments.
   */
  public void scan() {
    if (!controllersChanged()) {
      return;
    }

    // Clear all button bindings to prepare for rebinding
    CommandScheduler.getInstance().getDefaultButtonLoop().clear();

    int driverPort = -1;
    int operatorPort = -1;
    String driverType = "";
    String operatorType = "";
    String driverName = "Not Found";
    String operatorName = "Not Found";

    // --- Find and bind DRIVER controller ---
    // The outer loop iterates through configurations, respecting the order they were added
    // (priority)
    for (ControllerConfig config : controllerConfigs) {
      // Skip configs that aren't for the driver or the current robot mode
      if (config.controllerFunction != ControllerFunction.DRIVER
          || !config.modes.contains(Constants.currentMode)) {
        continue;
      }

      // The inner loop checks all controller ports for a name match
      for (int port = 0; port < NUM_CONTROLLER_PORTS; port++) {
        var controllerName = controllerNames[port];
        if (controllerNameMatchesType(controllerName, config.controllerType)) {
          driverPort = port;
          driverName = controllerName;
          driverType = config.controllerType.name();
          config.bindingCallback.accept(driverPort);
          break; // Found a match, stop searching ports
        }
      }

      if (driverPort != -1) {
        break; // Found and assigned a driver, stop searching configs
      }
    }

    Logger.recordOutput("Controller/DriverPort", driverPort);
    Logger.recordOutput("Controller/DriverType", driverType);
    Logger.recordOutput("Controller/DriverDevice", driverName);

    // --- Find and bind OPERATOR controller ---
    for (ControllerConfig config : controllerConfigs) {
      // Skip configs that aren't for the operator or the current robot mode
      if (config.controllerFunction != ControllerFunction.OPERATOR
          || !config.modes.contains(Constants.currentMode)) {
        continue;
      }

      // Check all ports for a name match, skipping the one used by the driver
      for (int port = 0; port < NUM_CONTROLLER_PORTS; port++) {
        if (port == driverPort) {
          continue; // Don't bind the same physical controller to both roles
        }
        var controllerName = controllerNames[port];
        if (controllerNameMatchesType(controllerName, config.controllerType)) {
          operatorPort = port;
          operatorName = controllerName;
          operatorType = config.controllerType.name();
          config.bindingCallback.accept(operatorPort);
          break; // Found a match, stop searching ports
        }
      }

      if (operatorPort != -1) {
        break; // Found and assigned an operator, stop searching configs
      }
    }

    Logger.recordOutput("Controller/OperatorPort", operatorPort);
    Logger.recordOutput("Controller/OperatorType", operatorType);
    Logger.recordOutput("Controller/OperatorDevice", operatorName);
  }
}
