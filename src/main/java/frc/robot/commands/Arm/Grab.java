package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class Grab extends CommandBase {
    private final Timer m_timer = new Timer();

    public static boolean done = false;

    @Override
    public void initialize() {
        m_timer.start();
        done = false;
    }

    @Override
    public void execute() {
        Functions.Grab();
        done = true;
    }

    @Override
    public void end(boolean interrupted) {
        done = true;
        Functions.Arm_extend(0);
        Functions.Stop_extend();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
