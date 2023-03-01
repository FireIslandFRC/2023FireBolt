package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.Variables;
import frc.robot.subsystems.*;

//this is drew
public class RaiseToTopCone extends CommandBase {

    public static boolean done = false;

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        if (RobotMap.Arm_Motor_Encoder.getPosition() < Variables.TopNodePosition) {
            Functions.Un_Brake();
            Functions.Arm_lift(Variables.ArmLiftSpeed);
        } else if (RobotMap.Arm_Motor_Encoder.getPosition() > Variables.TopNodePosition) {
            Functions.Brake();
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        done = true;
        Functions.Stop_lift();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
