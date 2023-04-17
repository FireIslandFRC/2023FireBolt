package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.Variables;
import frc.robot.subsystems.*;

//this is drew
public class LowerArm extends CommandBase {

    public static boolean done = false;

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {
        if (RobotMap.Arm_Motor_Encoder.getPosition() > 10) {
            Functions.Un_Brake();
            if (Functions.LimitUpDownValue()) {
                Functions.Arm_lift(Variables.ArmDropSpeed);
            } else if (!Functions.LimitUpDownValue()) {
                Functions.Stop_lift();
            }
        } else if (RobotMap.Arm_Motor_Encoder.getPosition() < 10) {
            Functions.Brake();
            Functions.Stop_lift();
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