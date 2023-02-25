package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
//this is drew
public class Brake extends CommandBase
{
    public static boolean done = false;

    public Brake(boolean set)
    {
        if(set){
            Functions.Brake();
            done = true;
        }else if(!set){
            Functions.Un_Brake();
            done = true;
        }
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {

    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        return done;
    }
}
