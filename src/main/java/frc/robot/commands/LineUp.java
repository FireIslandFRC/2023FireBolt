package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class LineUp extends CommandBase {
    private final Timer m_timer = new Timer();

    public static boolean done = false;

    @Override
    public void initialize() {
        m_timer.start();
        done = false;
    }

    @Override
    public void execute() {
        Functions.LineUp();
        done = true;
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
