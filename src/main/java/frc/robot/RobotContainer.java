// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Imports start here
//Welcome to the den of the beast...
package frc.robot;

import java.util.HashMap;
import java.util.List;

import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.commands.Arm.ArmOut;
import frc.robot.commands.Arm.ArmRest;
import frc.robot.commands.Arm.ArmGrabPos;
import frc.robot.commands.Arm.ArmRotateDown;
import frc.robot.commands.Arm.ArmRotateUp;
import frc.robot.commands.Arm.Drop;
import frc.robot.commands.Arm.Grab;
import frc.robot.commands.Arm.PullArmIn;
import frc.robot.commands.AutoCommands.ArmOutTop;
import frc.robot.commands.AutoCommands.ArmRetract;
import frc.robot.commands.AutoCommands.RaiseToBottom;
import frc.robot.commands.AutoCommands.RaiseToTopCone;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends TimedRobot {

  /* Controllers */
  private final static Joystick driver = new Joystick(0);
  private final static XboxController op = new XboxController(1);
  public final double auto = -1;

  /* Drive Controls */
  private final int translationAxis = Joystick.kDefaultYChannel;
  private final int strafeAxis = Joystick.kDefaultXChannel;
  private final int rotationAxis = Joystick.kDefaultTwistChannel;

  /* Driver Buttons */
  public static final JoystickButton zeroGyro = new JoystickButton(driver, 5);
  public static final JoystickButton robotCentric = new JoystickButton(driver, 2);
  public static final JoystickButton slowSpeed = new JoystickButton(driver, 1);
  public static final JoystickButton Grab = new JoystickButton(op, 7);
  public static final JoystickButton Drop = new JoystickButton(op, 8);
  public static final JoystickButton armlift = new JoystickButton(op, 1);
  public static final JoystickButton armlower = new JoystickButton(op, 2);
  public static final JoystickButton armout = new JoystickButton(op, 5);
  public static final JoystickButton armin = new JoystickButton(op, 6);
  public static final JoystickButton GrabDoubSub = new JoystickButton(op, 4);
  public static final JoystickButton BottomPick = new JoystickButton(op, 3);

  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public CommandBase meat = Commands.sequence(new PrintCommand("no auto"));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis) * 0.75,
            () -> -driver.getRawAxis(strafeAxis) * 0.75,
            () -> -driver.getRawAxis(rotationAxis) * 0.5,
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean()));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    armlower.whileTrue(new ArmRotateDown());
    armout.whileTrue(new ArmOut());
    armin.whileTrue(new PullArmIn());
    armlift.whileTrue(new ArmRotateUp());
    armlower.whileTrue(new ArmRotateDown());
    Grab.whileTrue(new Grab());
    Drop.whileTrue(new Drop());
    GrabDoubSub.onTrue(new ArmGrabPos());
    BottomPick.onTrue(new ArmGrabPos());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
   * public Command getAutonomousCommand() {
   * // An ExampleCommand will run in autonomous
   * return new exampleAuto(s_Swerve);
   * }
   */
  public Command getAutonomousCommand2(String pathName, HashMap<String, Command> eventMap) {

    // eventMap.put("event1", new ArmRotate());
    /*
     * Path spesific information. Getting paths defined by name and organizing
     * command groups
     */
    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathName, 2,        
    1);
    // defining variables used in thingy
    eventMap.put("raisearm", new RaiseToTopCone());
    eventMap.put("x", new ArmRest()); // matches x button

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
    // place cone/cube and then dock autonomous
    if (pathName.equals("ConeDock")) {
      meat = Commands.sequence(
          new Grab(),
          new RaiseToTopCone(),
          builder.fullAuto(path.get(0)),
          new ArmOutTop(),
          new Drop(),
          new ArmRetract(),
          builder.fullAuto(path.get(1)));
      // place cone/cube, pick up a second cone/cube then dock autonomous
    } else if (pathName.equals("ConePickDock")) {
      meat = Commands.sequence(
          new Grab(),
          new RaiseToTopCone(),
          new ArmOutTop(),
          new Drop(),
          new ArmRetract(),
          new ArmGrabPos(),
          builder.fullAuto(path.get(0)),
          new Grab(),
          builder.fullAuto(path.get(1)));
      // place cone/cube then taxi autonomous
    } else if (pathName.equals("Cone")) {
      meat = Commands.sequence(
          new Grab(),
          new RaiseToTopCone(),
          builder.fullAuto(path.get(0)),
          new ArmOutTop(),
          new Drop(),
          new ArmRetract(),
          builder.fullAuto(path.get(1)),
          new ArmRest());
      /* builder.fullAuto(path.get(0))); */
      // place cone/cube then pick up another cone/cube autonomous
    } else if (pathName.equals("ConePick")) {
      meat = Commands.sequence(
          new Grab(),
          new RaiseToTopCone(),
          new ArmOutTop(),
          new Drop(),
          new ArmRetract(),
          new ArmGrabPos(),
          builder.fullAuto(path.get(0)),
          new Grab());
    } else if (pathName.equals("taxi")) {
      meat = Commands.sequence(
          builder.fullAuto(path.get(0)));
    }else if (pathName.equals("MiddleConeDock")) {
      meat = Commands.sequence(
          new Grab(),
          new RaiseToTopCone(),
          builder.fullAuto(path.get(0)),
          new ArmOutTop(),
          new Drop(),
          new ArmRetract(),
          builder.fullAuto(path.get(1)));
    }else if (pathName.equals("Dock")) {
      meat = Commands.sequence(
          builder.fullAuto(path.get(0)));
    }else if (pathName.equals("Bottom")) {
      meat = Commands.sequence(
          new Grab(),
          new RaiseToBottom(),
          new Drop(),
          builder.fullAuto(path.get(0)));
      /* builder.fullAuto(path.get(0))); */
      // place cone/cube then pick up another cone/cube autonomous
    }else if (pathName.equals("BottomDock")) {
      meat = Commands.sequence(
          new Grab(),
          new RaiseToBottom(),
          new Drop(),
          builder.fullAuto(path.get(0)));
      /* builder.fullAuto(path.get(0))); */
      // place cone/cube then pick up another cone/cube autonomous
    }
    // returns the Meat of the auto
    return meat;
  }
}
