// package frc.robot.commands;

<<<<<<< HEAD
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Swerve;
=======
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Functions;
import frc.robot.subsystems.Swerve;
>>>>>>> parent of e81b77b (Potential LimeLight)

// import edu.wpi.first.wpilibj.Timer;

// import com.ctre.phoenix.sensors.Pigeon2;


// public class AutoLevel extends CommandBase {

//     public static boolean done = false;

//     private final Pigeon2 gyro;
    
//     private Double values;

//     public Drive m_drive = new Drive(new Swerve(), values, values, values);

//     public AutoLevel(){
//         gyro = new Pigeon2(30);
//     }

//   @Override
//   public void execute() {
//     if (gyro.getPitch() > 0){
//         values = 0.1;
//         m_drive.execute();
//         Timer.delay(0.3);
//     } else if (gyro.getPitch() < 0){
//         values = -0.1;
//         m_drive.execute();
//         Timer.delay(0.3);
//     } else if (gyro.getPitch() == 0){
//         values = 0.0;
//         m_drive.execute();
//         Timer.delay(0.3);
//         if(gyro.getPitch() == 0){
//         done = true;
//         };
//     }
//   }
//   @Override
//   public void end(boolean interrupted) {
//       done = true;
//   }

//   @Override
//   public boolean isFinished() {
//       return done;
//   }
// }
