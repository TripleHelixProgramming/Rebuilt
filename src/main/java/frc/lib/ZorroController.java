package frc.lib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;

public class ZorroController extends GenericHID implements Sendable {

  // RadioMaster Zorro joystick axis
  public enum Axis {
    kLeftXAxis(0),
    kLeftYAxis(1),
    kLeftDial(2),
    kRightDial(3),
    kRightXAxis(4),
    kRightYAxis(5);

    public final int value;

    Axis(int value) {
      this.value = value;
    }

    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }

  // RadioMaster Zorro buttons
  public enum Button {
    kBDown(1),
    kBMid(2),
    kBUp(3),
    kEDown(4),
    kEUp(5),
    kAIn(6),
    kGIn(7),
    kCDown(8),
    kCMid(9),
    kCUp(10),
    kFDown(11),
    kFUp(12),
    kDIn(13),
    kHIn(14);

    public final int value;

    Button(int value) {
      this.value = value;
    }

    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  /**
   * Construct an instance of a Zorro controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into (0-5).
   */
  public ZorroController(int port) {
    super(port);
  }

  public double getLeftXAxis() {
    return getRawAxis(Axis.kLeftXAxis.value);
  }

  public double getLeftYAxis() {
    return getRawAxis(Axis.kLeftYAxis.value);
  }

  public double getRightXAxis() {
    return getRawAxis(Axis.kRightXAxis.value);
  }

  public double getRightYAxis() {
    return getRawAxis(Axis.kRightYAxis.value);
  }

  public double getLeftDial() {
    return getRawAxis(Axis.kLeftDial.value);
  }

  public double getRightDial() {
    return getRawAxis(Axis.kRightDial.value);
  }

  public boolean getBDown() {
    return getRawButton(Button.kBDown.value);
  }

  public boolean getBMid() {
    return getRawButton(Button.kBMid.value);
  }

  public boolean getBUp() {
    return getRawButton(Button.kBUp.value);
  }

  public boolean getEDown() {
    return getRawButton(Button.kEDown.value);
  }

  public boolean getEUp() {
    return getRawButton(Button.kEUp.value);
  }

  public boolean getAIn() {
    return getRawButton(Button.kAIn.value);
  }

  public boolean getGIn() {
    return getRawButton(Button.kGIn.value);
  }

  public boolean getCDown() {
    return getRawButton(Button.kCDown.value);
  }

  public boolean getCMid() {
    return getRawButton(Button.kCMid.value);
  }

  public boolean getCUp() {
    return getRawButton(Button.kCUp.value);
  }

  public boolean getFDown() {
    return getRawButton(Button.kFDown.value);
  }

  public boolean getFUp() {
    return getRawButton(Button.kFUp.value);
  }

  public boolean getDIn() {
    return getRawButton(Button.kDIn.value);
  }

  public boolean getHIn() {
    return getRawButton(Button.kHIn.value);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "Zorro");

    builder.addDoubleProperty("LeftXAxis", this::getLeftXAxis, null);
    builder.addDoubleProperty("LeftYAxis", this::getLeftYAxis, null);
    builder.addDoubleProperty("LeftDial", this::getLeftDial, null);
    builder.addDoubleProperty("RightDial", this::getRightDial, null);
    builder.addDoubleProperty("RightXAxis", this::getRightXAxis, null);
    builder.addDoubleProperty("RightYAxis", this::getRightYAxis, null);

    builder.addBooleanProperty("AIn", this::getAIn, null);

    builder.addBooleanProperty("BDown", this::getBDown, null);
    builder.addBooleanProperty("BMid", this::getBMid, null);
    builder.addBooleanProperty("BUp", this::getBUp, null);

    builder.addBooleanProperty("CDown", this::getCDown, null);
    builder.addBooleanProperty("CMid", this::getCMid, null);
    builder.addBooleanProperty("CUp", this::getCUp, null);

    builder.addBooleanProperty("DIn", this::getDIn, null);

    builder.addBooleanProperty("EDown", this::getEDown, null);
    builder.addBooleanProperty("EUp", this::getEUp, null);

    builder.addBooleanProperty("FDown", this::getFDown, null);
    builder.addBooleanProperty("FUp", this::getFUp, null);

    builder.addBooleanProperty("GIn", this::getGIn, null);

    builder.addBooleanProperty("HIn", this::getHIn, null);
  }
}
