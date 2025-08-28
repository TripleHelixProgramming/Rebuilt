package frc.lib;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A version of {@link ZorroController} with {@link Trigger} factories for command-based.
 *
 * @see ZorroController
 */
@SuppressWarnings("MethodName")
public class CommandZorroController extends CommandGenericHID {
  private final ZorroController m_hid;

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into.
   */
  public CommandZorroController(int port) {
    super(port);
    m_hid = new ZorroController(port);
  }

  /**
   * Get the underlying GenericHID object.
   *
   * @return the wrapped GenericHID object
   */
  @Override
  public ZorroController getHID() {
    return m_hid;
  }

  /**
   * Constructs a Trigger instance around the B switch's down signal.
   *
   * @return a Trigger instance representing the B switch's down signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger BDown() {
    return BDown(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the B switch's down signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the B switch's down signal attached to the given loop.
   */
  public Trigger BDown(EventLoop loop) {
    return button(ZorroController.Button.kBDown.value, loop);
  }

  /**
   * Constructs a Trigger instance around the B switch's mid signal.
   *
   * @return a Trigger instance representing the B switch's mid signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger BMid() {
    return BMid(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the B switch's mid signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the B switch's mid signal attached to the given loop.
   */
  public Trigger BMid(EventLoop loop) {
    return button(ZorroController.Button.kBMid.value, loop);
  }

  /**
   * Constructs a Trigger instance around the B switch's up signal.
   *
   * @return a Trigger instance representing the B switch's up signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger BUp() {
    return BUp(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the B switch's up signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the B switch's up signal attached to the given loop.
   */
  public Trigger BUp(EventLoop loop) {
    return button(ZorroController.Button.kBUp.value, loop);
  }

  /**
   * Constructs a Trigger instance around the E switch's down signal.
   *
   * @return a Trigger instance representing the E switch's down signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger EDown() {
    return EDown(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the E switch's down signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the E switch's down signal attached to the given loop.
   */
  public Trigger EDown(EventLoop loop) {
    return button(ZorroController.Button.kEDown.value, loop);
  }

  /**
   * Constructs a Trigger instance around the E switch's up signal.
   *
   * @return a Trigger instance representing the E switch's up signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger EUp() {
    return EUp(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the E switch's up signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the E switch's up signal attached to the given loop.
   */
  public Trigger EUp(EventLoop loop) {
    return button(ZorroController.Button.kEUp.value, loop);
  }

  /**
   * Constructs a Trigger instance around the A button's in signal.
   *
   * @return a Trigger instance representing the A button's in signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger AIn() {
    return AIn(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the A button's in signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the A button's in signal attached to the given loop.
   */
  public Trigger AIn(EventLoop loop) {
    return button(ZorroController.Button.kAIn.value, loop);
  }

  /**
   * Constructs a Trigger instance around the G button's in signal.
   *
   * @return a Trigger instance representing the G button's in signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger GIn() {
    return GIn(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the G button's in signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the G button's in signal attached to the given loop.
   */
  public Trigger GIn(EventLoop loop) {
    return button(ZorroController.Button.kGIn.value, loop);
  }

  /**
   * Constructs a Trigger instance around the C switch's down signal.
   *
   * @return a Trigger instance representing the C switch's down signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger CDown() {
    return CDown(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the C switch's down signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the C switch's down signal attached to the given loop.
   */
  public Trigger CDown(EventLoop loop) {
    return button(ZorroController.Button.kCDown.value, loop);
  }

  /**
   * Constructs a Trigger instance around the C switch's mid signal.
   *
   * @return a Trigger instance representing the C switch's mid signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger CMid() {
    return CMid(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the C switch's mid signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the C switch's mid signal attached to the given loop.
   */
  public Trigger CMid(EventLoop loop) {
    return button(ZorroController.Button.kCMid.value, loop);
  }

  /**
   * Constructs a Trigger instance around the C switch's up signal.
   *
   * @return a Trigger instance representing the C switch's up signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger CUp() {
    return CUp(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the C switch's up signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the C switch's up signal attached to the given loop.
   */
  public Trigger CUp(EventLoop loop) {
    return button(ZorroController.Button.kCUp.value, loop);
  }

  /**
   * Constructs a Trigger instance around the F switch's down signal.
   *
   * @return a Trigger instance representing the F switch's down signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger FDown() {
    return FDown(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the F switch's down signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the F switch's down signal attached to the given loop.
   */
  public Trigger FDown(EventLoop loop) {
    return button(ZorroController.Button.kFDown.value, loop);
  }

  /**
   * Constructs a Trigger instance around the F switch's up signal.
   *
   * @return a Trigger instance representing the F switch's up signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger FUp() {
    return FUp(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the F switch's up signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the F switch's up signal attached to the given loop.
   */
  public Trigger FUp(EventLoop loop) {
    return button(ZorroController.Button.kFUp.value, loop);
  }

  /**
   * Constructs a Trigger instance around the D button's in signal.
   *
   * @return a Trigger instance representing the D button's in signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger DIn() {
    return DIn(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the D button's in signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the D button's in signal attached to the given loop.
   */
  public Trigger DIn(EventLoop loop) {
    return button(ZorroController.Button.kDIn.value, loop);
  }

  /**
   * Constructs a Trigger instance around the H button's in signal.
   *
   * @return a Trigger instance representing the H button's in signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #a(EventLoop)
   */
  public Trigger HIn() {
    return HIn(CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs a Trigger instance around the H button's in signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return a Trigger instance representing the H button's in signal attached to the given loop.
   */
  public Trigger HIn(EventLoop loop) {
    return button(ZorroController.Button.kHIn.value, loop);
  }

  /**
   * Get the X axis value of controller's left stick.
   *
   * @return The axis value.
   */
  public double getLeftXAxis() {
    return m_hid.getLeftXAxis();
  }

  /**
   * Get the X axis value of controller's right stick.
   *
   * @return The axis value.
   */
  public double getRightXAxis() {
    return m_hid.getRightXAxis();
  }

  /**
   * Get the Y axis value of controller's left stick.
   *
   * @return The axis value.
   */
  public double getLeftYAxis() {
    return m_hid.getLeftYAxis();
  }

  /**
   * Get the Y axis value of controller's right stick.
   *
   * @return The axis value.
   */
  public double getRightYAxis() {
    return m_hid.getRightYAxis();
  }

  /**
   * Get the axis value of controller's left dial.
   *
   * @return The axis value.
   */
  public double getLeftDial() {
    return m_hid.getLeftDial();
  }

  /**
   * Get the axis value of controller's right dial.
   *
   * @return The axis value.
   */
  public double getRightDial() {
    return m_hid.getRightDial();
  }
}
