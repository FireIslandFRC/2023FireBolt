package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.subsystems.*;

//this is drew
public class ArmOut extends CommandBase {
    private final Timer m_timer = new Timer();

    public static boolean done = false;

    @Override
    public void initialize() {
        m_timer.start();
        done = false;
    }

    @Override
    public void execute() {
        if(Constants.RobotMap.Arm_Motor_Encoder.getPosition() > 20){
            Functions.Arm_extend();
            }
            done = true;    }

    @Override
    public void end(boolean interrupted) {
        done = true;
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
