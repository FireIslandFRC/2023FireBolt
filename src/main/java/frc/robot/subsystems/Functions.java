package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.Timer;

public class Functions {
    private static final Timer m_timer = new Timer();

    public static void Drive_lift (double speed){
        RobotMap.Arm_Motor.set(speed);
    }
    public static void Drive_extend (double speed){
        RobotMap.Arm_Retract_Motor.set(speed);
    
    }

    public static void Stop_extend(double speed){
        RobotMap.Arm_Retract_Motor.set(speed);
    }
    public static void Stop_lift (){
        
        m_timer.start();
        Drive_lift(.3);
        if(m_timer.get() > 5){
 
        }
        
        


    }
    public static void Lift_To_3Height(double speed){
        
    }
    
}
