package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class ArmSpin extends CommandBase{
    private final Timer m_timer = new Timer();

    public static boolean done = false;

    @Override
    public void initialize(){
        m_timer.start();
        done = false;
    }

    @Override
    public void execute(){
        Arm.Drive(1);
        if (m_timer.get() > 3){
            m_timer.stop();
            m_timer.reset();
            done = true;
            Arm.Stop();
        }
    }

    @Override
    public void end(boolean interrupted){
        m_timer.stop();
            m_timer.reset();
            done = true;
            Arm.Stop();
    }

    @Override
    public boolean isFinished(){
        return done;
    }
}
