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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Arm.ArmRest;
import frc.robot.commands.Arm.ArmRestGrabPos;
import frc.robot.commands.Arm.ArmRotateDown;
import frc.robot.commands.Arm.ArmRotateUp;
import frc.robot.commands.Arm.Drop;
import frc.robot.commands.Arm.Grab;
import frc.robot.commands.AutoCommands.ArmOutTop;
import frc.robot.commands.AutoCommands.ArmRetract;
import frc.robot.commands.AutoCommands.RaiseToTopCone;
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
  private final static Joystick op = new Joystick(1);
  public final double auto = -1;
  /* Drive Controls */
  private final int translationAxis = Joystick.kDefaultYChannel;
  private final int strafeAxis = Joystick.kDefaultXChannel;
  private final int rotationAxis = Joystick.kDefaultTwistChannel;

  /* Driver Buttons */
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, 5);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, 2);
  private final JoystickButton slowSpeed =
      new JoystickButton(driver, 1);
  public static final JoystickButton Close = 
      new JoystickButton(op, 5);
  public static final JoystickButton Open =
      new JoystickButton(op, 10);
  public static final JoystickButton armlift =
      new JoystickButton(op, 7);
  public static final JoystickButton armlower =
      new JoystickButton(op, 8);
  public static final JoystickButton armout =
      new JoystickButton(op, 13);
  public static final JoystickButton armin =
      new JoystickButton(op, 14);

  
    
  

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  public CommandBase meat = Commands.sequence(new PrintCommand("not auto"));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis)*0.75,
            () -> -driver.getRawAxis(strafeAxis)*0.75,
            () -> -driver.getRawAxis(rotationAxis)*0.5,
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
    
    //eventMap.put("event1", new ArmRotate());

    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathName, 2,
                    3);

    eventMap.put("raisearm", new RaiseToTopCone());

            
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
    final ArmRotateDown armgo = new ArmRotateDown();



if (pathName.equals("ConeDock")){
      meat = Commands.sequence(
        new RaiseToTopCone(),
        new ArmOutTop(),
        new Drop(),
        new ArmRetract(),
        new ArmRest(),
        builder.fullAuto(path.get(0))
        );
}else if (pathName.equals("ConePickDock")){
      meat = Commands.sequence(
      new RaiseToTopCone(),
      new ArmOutTop(),
      new Drop(),
      new ArmRetract(),
      new ArmRestGrabPos(),
      builder.fullAuto(path.get(0)),
      new Grab(),
      builder.fullAuto(path.get(1)));
}else if (pathName.equals("Cone")){
      meat = Commands.sequence(
        new PrintCommand("potato code"),
        new RaiseToTopCone(),
        new ArmOutTop(),
        new Drop(),
        new ArmRetract(),
        new ArmRest(),
        builder.fullAuto(path.get(0))
      );
  }else if (pathName.equals("ConePick")){
    meat = Commands.sequence(
      new RaiseToTopCone(),
      new ArmOutTop(),
      new Drop(),
      new ArmRetract(),
      new ArmRest(),
      builder.fullAuto(path.get(0))
      );
    }
    //returns the Meat of the auto
    return meat;
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
