package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.Variables;
import frc.robot.subsystems.*;

//this is drew
public class PullArmIn extends CommandBase {
    private final Timer m_timer = new Timer();

    public static boolean done = false;

    @Override
    public void initialize() {
        m_timer.start();
        done = false;
    }

    @Override
    public void execute() {
            Functions.Arm_extend(-Variables.ArmExtendSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_timer.reset();
        done = true;
        Functions.Arm_extend(0);
        Functions.Stop_extend();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
