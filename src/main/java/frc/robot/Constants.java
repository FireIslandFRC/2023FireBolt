package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.lib.config.SwerveModuleConstants;
import frc.robot.commands.TeleopSwerve;

public final class Constants {

  public static final class RobotMap {
    public static CANSparkMax Arm_Motor = new CANSparkMax(10, MotorType.kBrushless);
    public static DoubleSolenoid Brake = new DoubleSolenoid(11, PneumaticsModuleType.REVPH, 1, 2);
    public static DoubleSolenoid Arm = new DoubleSolenoid(11, PneumaticsModuleType.REVPH, 14, 15);
    public static RelativeEncoder Arm_Motor_Encoder = Arm_Motor.getEncoder();
    public static DoubleSolenoid EndEffector = new DoubleSolenoid(11, PneumaticsModuleType.REVPH, 3, 4);
    public static DigitalInput LimitSwitchInOut = new DigitalInput(0);
    public static DigitalInput LimitSwitchUpDown = new DigitalInput(1);
    public static Pigeon2 gyro = new Pigeon2(30);

  }

  public static final class Variables {
    public static double ArmDropSpeed = -0.6;
    public static double ArmExtendSpeed = 1;
    public static double ArmRetractSpeed = -1;
    public static double ArmRestPosition = 10;
    public static double ArmRestGrabPosition = 20;
    public static double ArmLiftSpeed = 0.6;
    public static double ArmRetractPostion = 4;
    public static double ArmExtendPostion = 5;
    public static double TopNodePosition = 80;
    public static double GrabDoubSubPos = 80;
    public static double xvalue;
    public static double yvalue;
    public static double twistvalue;
  }

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Pigeon */
    public static final double pigeon = 30;
    public static final Pigeon2 gyro = new Pigeon2(30);
    

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(28);
    public static final double wheelBase = Units.inchesToMeters(26);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.75:1
    public static final double angleGearRatio = (150.0 / 7.0); //

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second //4.5
    public static final double maxAngularVelocity = 6; // 11.5

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 21;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(34 + 180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 22;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(110 + 180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 23;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(170 + 180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 24;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(59 + 180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    public void setDefaultCommand(TeleopSwerve teleopSwerve) {
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4; // 3
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
