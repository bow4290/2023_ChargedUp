package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class GenericGamepad {
  /** Equivalent to × Blue Cross on PS4 */
  public Trigger cross_a;
  /** Equivalent to ○ Red Circle on PS4 */
  public Trigger circle_b;
  /** Equivalent to □ Purple Square on PS4 */
  public Trigger square_x;
  /** Equivalent to △ Green Triangle on PS4 */
  public Trigger triangle_y;

  // Also known as L1 on PS4
  public Trigger leftBumper;
  // Also known as
  public Trigger rightBumper;

  public Trigger dpadUp;
  public Trigger dpadLeft;
  public Trigger dpadRight;
  public Trigger dpadDown;

  public DoubleSupplier leftTrigger;
  public DoubleSupplier rightTrigger;
  // Warning: leftY is -1 when up
  public DoubleSupplier leftY;
  public DoubleSupplier leftX;
  // Warning: rightY is -1 when up
  public DoubleSupplier rightY;
  public DoubleSupplier rightX;

  public Trigger leftJoystickPushed;
  public Trigger rightJoystickPushed;
  // Equivalent to Back on Xbox or Share on PS4
  public Trigger leftMiddle;
  // Equivalent to Start on Xbox or Options on PS4
  public Trigger rightMiddle;
  // Equivalent to Touchpad on PS4 (NO EQUIVALENT ON XBOX)
  public Trigger topMiddle;
  // Equivalent to Options on PS4 (NO EQUIVALENT ON XBOX)
  public Trigger bottomMiddle;

  public Trigger leftTriggerB;
  public Trigger rightTriggerB;

  public DoubleConsumer rumble;

  public static GenericGamepad from(int port, boolean isPS4) {
    if (isPS4) {
      return new GenericGamepad(new CommandPS4Controller(port));
    } else {
      return new GenericGamepad(new CommandXboxController(port));
    }
  }

  public GenericGamepad(CommandPS4Controller controller) {
    cross_a = controller.cross();
    circle_b = controller.circle();
    square_x = controller.square();
    triangle_y = controller.triangle();

    leftBumper = controller.L1();
    rightBumper = controller.R1();
    leftTrigger = controller::getL2Axis;
    rightTrigger = controller::getR2Axis;
    leftTriggerB = controller.L2();
    rightTriggerB = controller.R2();

    leftY = controller::getLeftY;
    leftX = controller::getLeftX;
    rightY = controller::getRightY;
    rightX = controller::getRightX;

    leftJoystickPushed = controller.L3();
    rightJoystickPushed = controller.R3();

    dpadUp = controller.povUp();
    dpadLeft = controller.povLeft();
    dpadRight = controller.povRight();
    dpadDown = controller.povDown();

    leftMiddle = controller.share();
    rightMiddle = controller.options();

    topMiddle = controller.touchpad();
    bottomMiddle = controller.PS();

    rumble = (val) -> {};
  }

  public GenericGamepad(CommandXboxController controller) {
    cross_a = controller.a();
    circle_b = controller.b();
    square_x = controller.x();
    triangle_y = controller.y();

    leftBumper = controller.leftBumper();
    rightBumper = controller.rightBumper();
    leftTrigger = controller::getLeftTriggerAxis;
    rightTrigger = controller::getRightTriggerAxis;
    rightTriggerB = controller.rightTrigger();
    leftTriggerB = controller.leftTrigger();

    leftY = controller::getLeftY;
    leftX = controller::getLeftX;
    rightY = controller::getRightY;
    rightX = controller::getRightX;

    leftJoystickPushed = controller.leftStick();
    rightJoystickPushed = controller.rightStick();

    dpadUp = controller.povUp();
    dpadLeft = controller.povLeft();
    dpadRight = controller.povLeft();
    dpadDown = controller.povDown();

    leftMiddle = controller.back();
    rightMiddle = controller.start();

    topMiddle = new Trigger(() -> false);
    bottomMiddle = new Trigger(() -> false);

    rumble = (val) -> controller.getHID().setRumble(RumbleType.kBothRumble, val);
  }
}
