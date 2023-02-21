// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  
  /* Controllers */
  private final static Joystick driver = new Joystick(0);

  /* Drive Controls */
  private final int translationAxis = Joystick.kDefaultYChannel;
  private final int strafeAxis = Joystick.kDefaultXChannel;
  private final int rotationAxis = Joystick.kDefaultTwistChannel;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, Joystick.ButtonType.kTrigger.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  private final JoystickButton slowSpeed =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);

      public static final JoystickButton armlift =
      new JoystickButton(driver, 7 );

  

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis)*0.5,
            () -> -driver.getRawAxis(strafeAxis)*0.5,
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }*/
  public Command getAutonomousCommand2(String pathName, HashMap<String, Command> eventMap) {
    eventMap.put("event1", new ArmSpin());

    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathName, 1,
                    3);

            
    List<PathPlannerTrajectory> autopath1 = 
    PathPlanner.loadPathGroup(pathName, 1, 5);

    SwerveAutoBuilder builder = new SwerveAutoBuilder(
                    s_Swerve::getPose,
                    s_Swerve::resetOdometry,
                    Constants.Swerve.swerveKinematics,
                    new PIDConstants(1.5, 0, 0),
                    new PIDConstants(3, 0, 0),
                    s_Swerve::setModuleStates,
                    eventMap,
                    true,
                    s_Swerve);
    final ArmSpin armgo = new ArmSpin();
    return Commands.sequence(
      builder.fullAuto(path.get(0)),
      new PrintCommand("start"),
      armgo,
      new PrintCommand("done"),
      builder.fullAuto(path.get(1))
    );
}
/*public Command getAutonomousCommand3(String pathName, HashMap<String, Command> eventMap) {
  eventMap.put("event1", new ArmSpin());

  List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, 1, 3);

          
  List<PathPlannerTrajectory> autopath1 = 
  PathPlanner.loadPathGroup("testpath1", 1, 1);

  return Commands.sequence(
    new PathFollowingCommand(),
    new PathFollowingCommand()
  );
}*/

}
