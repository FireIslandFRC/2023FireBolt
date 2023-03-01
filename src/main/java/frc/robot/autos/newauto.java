package frc.robot.autos;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
//import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class newauto extends SequentialCommandGroup{
    public newauto(Swerve s_Swerve){
        PathPlannerTrajectory newPath1 = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));

        PathPlannerState exampleState = (PathPlannerState) newPath1.sample(1.2);

        System.out.println(exampleState.velocityMetersPerSecond);

        /*PathPlannerTrajectory traj1 = PathPlanner.generatePath(
    new PathConstraints(4, 3), 
    new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading
    new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45)) // position, heading
);*/

var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(0, 2*Math.PI);

    /*PPSwerveControllerCommand swerveControllerCommand1 =
        new PPSwerveControllerCommand(
            traj1,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            new PIDController(Constants.AutoConstants.kPThetaController, 0, 0),
            s_Swerve::setModuleStates,
            s_Swerve);*/
}
}