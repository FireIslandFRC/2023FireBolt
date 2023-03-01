package frc.robot.subsystems;

import frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Functions {

    public static void Arm_lift(double speed1) {
        RobotMap.Arm_Motor.set(speed1);
    }

    public static void Arm_extend(double speed) {
        RobotMap.Arm_Extend_Motor.set(speed);
    }

    public static void Stop_extend() {
        RobotMap.Arm_Extend_Motor.set(0);
    }

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
        RobotMap.EndEffector.set(Value.kReverse);
    }

    public static void Release() {
        RobotMap.EndEffector.set(Value.kForward);
    }

    public static boolean LimitValue() {
        boolean value = RobotMap.LimitSwitch.get();
        return value;
    }

}
