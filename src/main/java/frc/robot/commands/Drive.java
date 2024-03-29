package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Drive extends CommandBase {
  private Swerve s_Swerve;
  private Double translationSup;
  private Double strafeSup;
  private Double rotationSup;
  public Drive(
      Swerve s_Swerve,
      Double translationSup,
      Double strafeSup,
      Double rotationSup) {
    this.s_Swerve = s_Swerve;

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
  }

  @Override
  public void execute() {

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationSup, strafeSup).times(Constants.Swerve.maxSpeed),
        rotationSup * Constants.Swerve.maxAngularVelocity,
        false,
        true);
  }
}
