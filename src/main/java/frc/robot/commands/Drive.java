package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.TeleopSwerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Drive extends CommandBase {
  private Swerve s_Swerve;
  private Double translationSup;
  private Double strafeSup;
  private Double rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowSpeedSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(4.0);

  public Drive(
      Swerve s_Swerve,
      Double translationSup,
      Double strafeSup,
      Double rotationSup) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
  }

  @Override
  public void execute() {

    /* Drive */
  }
}
