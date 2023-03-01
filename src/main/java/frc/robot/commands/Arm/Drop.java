package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

//this is drew
public class Drop extends CommandBase {

    public static boolean done = false;

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Functions.Release();
        done = true;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
// i didnt mess with anything ;)