package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

//this is drew
public class PullArmOff extends CommandBase {

    public static boolean done = false;

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
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
