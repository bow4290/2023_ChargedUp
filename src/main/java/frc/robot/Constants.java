package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static final double stickDeadband = 0.1;
  public static final double driveSens = 1.0;
  public static final double turnSens = 1.0;
  public static final boolean enablePS5 = false;

  public static final class Limelight {

    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(
                -0.082,
                -0.2105,
                // 0,
                0.58),
            new Rotation3d(0, 0, 0));
    public static final String camName = "OV5647";
  }

  public static final class Swerve {
    public static final int pigeonID = 1;

    public static final COTSFalconSwerveConstants chosenModule =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Drivetrain Constants */
    // Approximate-ish
    // Updated for 29x29 robot (contemplate: a 29x29 bot will not have a distance of 29 inches
    // between the wheels)
    public static final double trackWidth = Units.inchesToMeters(23.75);
    public static final double wheelBase = Units.inchesToMeters(23.75);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = chosenModule.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 40;
    public static final int drivePeakCurrentLimit = 45;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double driveKS = (0.106 / 12); // TODO: This must be tuned to specific robot
    public static final double driveKV = (2.394 / 12);
    public static final double driveKA = (0.304 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    // FYI this does not change teleop behavior only auto (or does it ???)
    // Measured 2023-02-25
    public static final double maxSpeed = 4.0;
    // WARNING IF YOU TURN THIS TOO HIGH IT TIPS THE BOT AND EXPLODES
    public static final double maxAngularVelocity = 5.0;

    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(295.13);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(114.08);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(60.02);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(193.27);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class Intake {
    public static final int leftIntakeID = 13;
    public static final int rightIntakeID = 12;
    public static final int solenoidPortForward = 2;
    public static final int solenoidPortReverse = 0;
    public static final PneumaticsModuleType pneumaticType = PneumaticsModuleType.CTREPCM;

    public static final double inSpeed = -1;
    public static final double ejectSpeed = 0.3;
    public static final double stopSpeed = 0;
  }

  public static final class Elbow {
    public static final int elbowPivotID = 9;
    public static final int elbowPivot2ID = 11;

    public static final double motionVelocity = 14000; // 10000;
    public static final double motionAcceleration = 18000; // 7500;
    public static final int motionSmoothing = 1;

    // 512/7 ratio, according to build
    // Testing determined the ratio is probably closer to 1024/7 so just use that
    // Further testing discovered that is not the actual gear ratio so be aware that it is an
    // approximation
    // Build also said that the gear ratio might be 128 / 1; this would be more accurate but it
    // would mess up all the values that we had tuned so just leave it for now
    // public static final double gearRatio = 1024 / 7;
    public static final double gearRatio = 128;
    public static final double talonCPR = 2048;
    public static final double ticksPerDegree = gearRatio * talonCPR / 360;
    public static final double degreesPerTick = 1 / ticksPerDegree;

    public static final double elbowDeadband = 0.02;

    public static double rotationEps = 2 * ticksPerDegree;
    public static double velocityEps = 6 * ticksPerDegree;

    public static final double kP = 0.5; // 0.5;
    public static final double kD = 2.5; // 5;
    public static final double kF = 0.04; // 0.04;

    public static double forwardLimit = 60;
    public static double backwardLimit = -110;

    public static double autoTimeout = 2.5;
  }

  public static final class Elevator {
    public static double positionEps = 1000;
    public static double velocityEps = 4000;
    public static final int elevatorMotorID = 10;
    public static final double extendSpeed = 0.3;
    public static final double retractSpeed = -0.3;
    public static final double stopSpeed = 0;

    public static final double elevatorDeadband = 0.03;
    // 8740
    public static final double base = 0;
    public static final double middle = 50000;
    public static final double max = 82500;

    public static final double revolutionsPerMeter = 40 * 2048; // (APPROXIMATE, DO NOT USE)
    public static final double metersPerRevolution = 1 / revolutionsPerMeter;

    // Motion magic stuff
    public static final double motionVelocity = 15000;
    public static final double motionAcceleration = 20000;
    public static final int motionSmoothing = 2;
    public static final double kP = 0.3;
    public static final double kD = 2;
    public static final double kF = 0.058;

    public static final double autoTimeout = 2;
  }

  public static final
  class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
    // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 10; // 1.83 4.42
    public static final double kPYController = 10;
    public static final double kPThetaController = 6;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
