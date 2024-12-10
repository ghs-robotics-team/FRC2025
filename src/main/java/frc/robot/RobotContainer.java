// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;


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
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
    //SwerveDrive swerveDrive=new SwerveParser(new File(Filesystem.getDeployDirectory(),"swerve")).createSwerveDrive(Units.feetToMeters(14.5));

    driverXbox = new XboxController(0);
    driverController = new XboxController(1);
    Command driveCommand = null;
    driveCommand = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY()*0.25, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX()*0.25, OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-driverXbox.getRightX()*0.25, OperatorConstants.RIGHT_X_DEADBAND));
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
//    new Trigger(m_exampleSubsystem::exampleCondition)
//        .onTrue(new ExampleCommand(m_exampleSubsystem));
    new JoystickButton(driverXbox, 7).onTrue((new InstantCommand(drivebase::zeroGyro)));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
//    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
//    return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
