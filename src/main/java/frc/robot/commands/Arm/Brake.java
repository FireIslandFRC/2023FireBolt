package frc.robot.commands.Arm;

import java.util.function.Function;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Variables;
import frc.robot.subsystems.*;

//this is drew
public class Brake extends CommandBase {

    public static boolean done = false;

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        Functions.Stop_lift();
        Functions.Brake();
    }

    @Override
    public void end(boolean interrupted) {
        Functions.Stop_lift();
        Functions.Brake();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
