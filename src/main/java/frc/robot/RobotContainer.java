package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick driverPS5 = new Joystick(1);

    /* Drive Controls */
    private final int forwardBackwardAxis = XboxController.Axis.kLeftY.value;
    private final int leftRightAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int forwardBackwardAxisPS5 = PS4Controller.Axis.kLeftY.value;
    private final int leftRightAxisPS5 = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxisPS5 = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton zeroGyroPS5 = new JoystickButton(driverPS5, PS4Controller.Button.kTriangle.value);

    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton robotCentricPS5 = new JoystickButton(driverPS5, PS4Controller.Button.kL1.value);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();


    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(forwardBackwardAxis) - driverPS5.getRawAxis(forwardBackwardAxisPS5), 
                () -> -driver.getRawAxis(leftRightAxis) - driverPS5.getRawAxis(leftRightAxisPS5), 
                () -> -driver.getRawAxis(rotationAxis) - driverPS5.getRawAxis(rotationAxisPS5), 
                () -> robotCentric.getAsBoolean() || robotCentricPS5.getAsBoolean()
            )
        );

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        zeroGyroPS5.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    }

    public Command getAutonomousCommand() {
        return new exampleAuto(s_Swerve);
    }
}
