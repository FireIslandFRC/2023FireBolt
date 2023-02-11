package frc.robot.subsystems;

import frc.robot.Constants.*;

public class Arm {

    public static void Drive (double speed){
        RobotMap.Arm_Motor.set(speed);
    }

    public static void Stop (){
        RobotMap.Arm_Motor.set(0);
    }
    
}
