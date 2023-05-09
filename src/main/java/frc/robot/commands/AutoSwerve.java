package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AutoSwerve extends CommandBase {
  private Swerve s_Swerve;
  private Double translationSup;
  private Double strafeSup;
  private Double rotationSup;
  private Boolean robotCentricSup;
  private Boolean slowSpeedSup;
  private Boolean autoLevel;

  private double translationVal = 0;
  private double strafeVal = 0;
  private double rotationVal = 0;

  public static boolean done = false;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.0);

  public AutoSwerve(
      Swerve s_Swerve,
      Double translationSup,
      Double strafeSup,
      Double rotationSup,
      Boolean robotCentricSup,
      Boolean slowSpeedSup,
      Boolean autoLevel) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.slowSpeedSup = slowSpeedSup;
    this.autoLevel = autoLevel;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    double speedMultiplier = slowSpeedSup ? 1.3 : 1.0;

    /* Translation Values */
    if (autoLevel) {
      if (Constants.Swerve.gyro.getPitch() < -11){
        translationVal = -0.15;
      }else if (Constants.Swerve.gyro.getPitch() > 11){
        translationVal = 0.15;
      }else {
        translationVal = 0;
      }

      System.out.println("***************Auto Leveling***************");
    } else {
      translationVal = translationLimiter.calculate(
          speedMultiplier * MathUtil.applyDeadband(translationSup, Constants.Swerve.stickDeadband));
    }

    /* Strafe Value */
    if (autoLevel) {
      strafeVal = 0;
      new PrintCommand("strafeVal");
    } else {
      strafeVal = strafeLimiter.calculate(
          speedMultiplier * MathUtil.applyDeadband(strafeSup, Constants.Swerve.stickDeadband));
    }

    /* Rotation Value*/
    if (autoLevel) {
      rotationVal = 0;
      new PrintCommand("rotationVal");
    } else {
      rotationVal = rotationLimiter.calculate(
          speedMultiplier * MathUtil.applyDeadband(rotationSup, Constants.Swerve.stickDeadband));
    }

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup,
        true);
  }
  @Override
    public boolean isFinished() {
        return done;
    }
}
