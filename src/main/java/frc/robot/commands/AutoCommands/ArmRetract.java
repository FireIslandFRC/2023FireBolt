package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.Variables;
import frc.robot.subsystems.*;

//this is drew
public class ArmRetract extends CommandBase {
    private final Timer m_timer = new Timer();

    public static boolean done = false;

    @Override
    public void initialize() {
        m_timer.start();
        done = false;
    }

    @Override
    public void execute() {
        if (RobotMap.Arm_Extend_Motor_Encoder.getPosition() > Variables.ArmRetractPostion) {
            Functions.Arm_extend(Variables.ArmRetractSpeed);
        } else if (RobotMap.Arm_Extend_Motor_Encoder.getPosition() <= Variables.ArmRetractPostion) {
            Functions.Stop_extend();
            done = true;
        }
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
