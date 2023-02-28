package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class GenericGamepad {
  // Equivalent to × Blue Cross on PS4
  public Trigger a;
  // Equivalent to ○ Red Circle on PS4
  public Trigger b;
  // Equivalent to □ Purple Square on PS4
  public Trigger x;
  // Equivalent to △ Green Triangle on PS4 (yes, this is slightly confusing)
  public Trigger y;
  public Trigger leftBumper;
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

  public Trigger leftTriggerB;
  public Trigger rightTriggerB;

  public GenericGamepad(CommandPS4Controller controller) {
    a = controller.cross();
    b = controller.circle();
    x = controller.square();
    y = controller.triangle();

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
  }

  public GenericGamepad(CommandXboxController controller) {
    a = controller.a();
    b = controller.b();
    x = controller.x();
    y = controller.y();

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
  }
}
