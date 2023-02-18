package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ArmSpin extends CommandBase{
    private final Timer m_timer = new Timer();

    public static boolean done = false;

    @Override
    public void initialize(){
        m_timer.reset();
    }

    @Override
    public void execute(){
        
        m_timer.start();
        System.out.println("potato");
        if (m_timer.get() > 3){
            end(true);
        }
    }

    @Override
    public void end(boolean interrupted){
        Arm.Stop();
    }

    @Override
    public boolean isFinished(){
        Arm.Stop();
        return done;
    }
}
