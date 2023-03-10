package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.*;
import frc.robot.Constants.Variables;

public class ArmRestGrabPos extends CommandBase {

    public static boolean done = false;

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (RobotMap.Arm_Motor_Encoder.getPosition() < Variables.ArmRestGrabPosition) {
            Functions.Un_Brake();
            Functions.Arm_lift(0.6);
        } else if (RobotMap.Arm_Motor_Encoder.getPosition() >= Variables.ArmRestGrabPosition) {
            Functions.Brake();
            Functions.Stop_lift();
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
