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

public class TeleopSwerve extends CommandBase {
  private Swerve s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowSpeedSup;
  private BooleanSupplier autoLevel;

  private double translationVal = 0;
  private double strafeVal = 0;
  private double rotationVal = 0;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.0);

  public TeleopSwerve(
      Swerve s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier slowSpeedSup,
      BooleanSupplier autoLevel) {
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

    double speedMultiplier = slowSpeedSup.getAsBoolean() ? 1.3 : 1.0;

    /* Translation Values */
    if (autoLevel.getAsBoolean()) {
      if (Constants.Swerve.gyro.getPitch() < -12){
        translationVal = 0.15;
      }else if (Constants.Swerve.gyro.getPitch() > 12){
        translationVal = -0.15;
      }else {
        translationVal = 0;
      }

      System.out.println("lkjfahhsakjdsakjflsaksafdhlkasdfhlkjasdfasdghklsadfhjkjgsa");
    } else {
      translationVal = translationLimiter.calculate(
          speedMultiplier * MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    }

    /* Strafe Value */
    if (autoLevel.getAsBoolean()) {
      strafeVal = 0;
      new PrintCommand("strafeVal");
    } else {
      strafeVal = strafeLimiter.calculate(
          speedMultiplier * MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Swerve.stickDeadband));
    }

    /* Rotation Value*/
    if (autoLevel.getAsBoolean()) {
      rotationVal = 0;
      new PrintCommand("rotationVal");
    } else {
      rotationVal = rotationLimiter.calculate(
          speedMultiplier * MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.Swerve.stickDeadband));
    }

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
        rotationVal * Constants.Swerve.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        true);
  }
}
