package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Variables;
import frc.robot.subsystems.*;

//this is drew
public class ArmRotateUp extends CommandBase {

    public static boolean done = false;

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        Functions.Un_Brake();
        Functions.Arm_lift(Variables.ArmLiftSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        Functions.Stop_lift();
        Functions.Brake();
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
