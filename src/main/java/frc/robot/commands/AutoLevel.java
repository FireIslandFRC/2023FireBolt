package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;

//import com.ctre.phoenix.sensors.Pigeon2;

 
public class AutoLevel extends CommandBase {

    public static boolean done = false;

    
    private Double values;

    public Drive m_drive = new Drive(new Swerve(), values, values, values);

    Swerve s_Swerve;

    public AutoLevel(){
    }

  @Override
  public void execute() {
    /*if (RobotMap.gyro.getPitch() > 0){
        values = 0.1;
        m_drive.execute();
        Timer.delay(0.3);
    } else if (RobotMap.gyro.getPitch() < 0){
        values = -0.1;
        m_drive.execute();
        Timer.delay(0.3);
    } else if (RobotMap.gyro.getPitch() == 0){
        values = 0.0;
        m_drive.execute();
        Timer.delay(0.3);
        if(RobotMap.gyro.getPitch() == 0){
            done = true;
        }
    }*/
    if (RobotMap.gyro.getPitch() > 0){
        s_Swerve.drive(
        new Translation2d(0.5, 0).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        true,
        true);
        Timer.delay(0.3);
        s_Swerve.drive(
        new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        true,
        true);
    } else if (RobotMap.gyro.getPitch() < 0){
        s_Swerve.drive(
        new Translation2d(-0.5, 0).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        true,
        true);
        Timer.delay(0.3);
        s_Swerve.drive(
        new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        true,
        true);
    } else if (RobotMap.gyro.getPitch() == 0){
        s_Swerve.drive(
        new Translation2d(0.5, 0).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        true,
        true);
        Timer.delay(0.3);
        s_Swerve.drive(
        new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
        0 * Constants.Swerve.maxAngularVelocity,
        true,
        true);
        if(RobotMap.gyro.getPitch() == 0){
            done = true;
        }
    }
  }
  @Override
  public void end(boolean interrupted) {
      done = true;
  }

  @Override
  public boolean isFinished() {
      return done;
  }
}
