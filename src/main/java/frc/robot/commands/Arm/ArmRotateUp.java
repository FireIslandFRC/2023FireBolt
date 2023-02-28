package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Variables;
import frc.robot.subsystems.*;
//this is drew
public class ArmRotateUp extends CommandBase{
    private final Timer m_timer = new Timer();

    public static boolean done = false;

    @Override
    public void initialize(){
        m_timer.start();
        done = false;
    }

    @Override
    public void execute(){
        Functions.Un_Brake();
        Functions.Arm_lift(Variables.ArmLiftSpeed);
    }

    @Override
    public void end(boolean interrupted){
        Functions.Brake();
        m_timer.stop();
        m_timer.reset();
        done = true;
        Functions.Stop_lift();
    }

    @Override
    public boolean isFinished(){
        return done;
    }
}
