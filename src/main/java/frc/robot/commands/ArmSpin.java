package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ArmSpin extends CommandBase{

    public static boolean done = false;

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        Arm.Drive(5);
    }

    @Override
    public void end(boolean interrupted){
        Arm.Stop();
    }

    @Override
    public boolean isFinished(){
        return done;
    }
}
