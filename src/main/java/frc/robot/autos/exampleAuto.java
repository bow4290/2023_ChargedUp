package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.Swerve;
import java.util.HashMap;

public class exampleAuto extends SequentialCommandGroup {
  public exampleAuto(Swerve s_Swerve, Elbow s_Elbow) {
    var pathGroup = PathPlanner.loadPathGroup("Best Path", new PathConstraints(3, 3));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put(
        "thirdTierCone",
        s_Elbow
            .posCmd(0)
            .until(
                () ->
                    Math.abs(s_Elbow.getPosition())
                        < 100.0)); // TODO Find out a better way to do this

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every
    // time you want to create an auto command. A good place to put this is in RobotContainer along
    // with your subsystems.
    SwerveAutoBuilder autoBuilder =
        new SwerveAutoBuilder(
            s_Swerve::getPose, // Pose2d supplier
            s_Swerve
                ::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(Constants.AutoConstants.kPYController, 0.0, 0.0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
            s_Swerve::setModuleStates,
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color.
            // Optional, defaults to true
            s_Swerve // The drive subsystem. Used to properly set the requirements of path following
            // commands
            );

    Command fullAuto = autoBuilder.fullAuto(pathGroup);

    addCommands(fullAuto);
    addRequirements(s_Elbow);
  }
}
