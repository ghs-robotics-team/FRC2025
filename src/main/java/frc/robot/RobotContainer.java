// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.TargetPoints;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.EagleEyeCommand;
import frc.robot.subsystems.EagleEye;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController driverController;
  XboxController driverXbox;
  Joystick rightjoystick;
  Joystick leftjoystick;
  private final EagleEye eagleye = new EagleEye();
  private final EagleEyeCommand eagleeyecommand = new EagleEyeCommand(eagleye);

  private final DriveToPointCommand driveTopLeft = new DriveToPointCommand(TargetPoints.TOP_LEFT);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
    eagleye.setDefaultCommand(eagleeyecommand);
    //SwerveDrive swerveDrive=new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Units.feetToMeters(14.5));

    if (Constants.OperatorConstants.XBOX_DRIVE)
    {
      driverXbox = new XboxController(0); 
      driverController = new XboxController(1);
    } else
    {
      rightjoystick = new Joystick(0);
      leftjoystick = new Joystick(1);
      driverController = new XboxController(2);
    }

    Command driveCommand = null;
    if (OperatorConstants.XBOX_DRIVE)
    {
      driveCommand = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));
    } else
    {
      driveCommand = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(leftjoystick.getRawAxis(1)*0.25, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(leftjoystick.getRawAxis(0)*0.25, OperatorConstants.LEFT_X_DEADBAND),
        () -> -rightjoystick.getRawAxis(0)*0.25);
    }
    configureBindings();

    drivebase.setDefaultCommand(driveCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    if (OperatorConstants.XBOX_DRIVE)
    {
      new JoystickButton(driverXbox, 2).onTrue(driveTopLeft); //B
      //new JoystickButton(driverXbox, 3).onTrue(new DriveToPointCommand(TargetPoints.TOP_RIGHT));
      new JoystickButton(driverXbox, 7).onTrue((new InstantCommand(drivebase::zeroGyro))); //Back Button
      new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(() -> {
        CommandScheduler.getInstance().cancelAll();
        /*if (CommandScheduler.getInstance().isScheduled(driveTopLeft)) {
            CommandScheduler.getInstance().cancel(driveTopLeft);
            System.out.println("driveTopLeft canceled.");
        } else {
            System.out.println("driveTopLeft is not running.");
        }
        */}));
     //X
    } 
  
    else
    {

    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
