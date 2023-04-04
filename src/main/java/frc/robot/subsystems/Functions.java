package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Functions {

    public static void Arm_lift(double speed1) {
        RobotMap.Arm_Motor.set(speed1);
    }

    public static void Arm_extend() {
        //RobotMap.Arm_Extend_Motor.set(speed);
        RobotMap.Arm_Extend_Piston_1.set(Value.kForward);
        RobotMap.Arm_Extend_Piston_2.set(Value.kForward);
    }

    public static void Arm_Retract() {
        /*if(LimitInOutValue()){
        RobotMap.Arm_Extend_Motor.set(-speed);
        }else{
            Stop_extend();
        }*/
        RobotMap.Arm_Extend_Piston_1.set(Value.kReverse);
        RobotMap.Arm_Extend_Piston_2.set(Value.kReverse);
    }

    // public static void Stop_extend() {
    //     RobotMap.Arm_Extend_Motor.set(0);
    // }

    public static void Stop_lift() {
        RobotMap.Arm_Motor.set(0);
    }

    public static void Brake() {
        RobotMap.Brake.set(Value.kReverse);
    }

    public static void Un_Brake() {
        RobotMap.Brake.set(Value.kForward);
    }

    public static void Grab() {
        RobotMap.EndEffector.set(Value.kForward);
    }

    public static void Release() {
        RobotMap.EndEffector.set(Value.kReverse);
    }

    public static boolean LimitInOutValue() {
        boolean valueInOut = RobotMap.LimitSwitchInOut.get();
        return valueInOut;
    }

    public static boolean LimitUpDownValue() {
            boolean valueUpDown = RobotMap.LimitSwitchUpDown.get();
            return valueUpDown;
    }

}
