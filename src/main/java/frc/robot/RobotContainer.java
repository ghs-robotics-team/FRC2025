package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.TargetPoints;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.EagleEyeCommand;
import frc.robot.commands.IntakeCommand;
//import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EagleEye;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.ArmSetpoint;
import frc.robot.commands.DriveLocalCommandAbsolute;
import frc.robot.commands.NearestTag;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ArmSetpoint;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ArmSteady;

public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem drivebase;
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator(); // ALWAYS SET AT BOTTOM FOR CORRECT ENCODER POS
  private final EagleEye eagleye = new EagleEye();
  private final EagleEyeCommand eagleeyecommand = new EagleEyeCommand(eagleye);

  // Controllers
  private Joystick buttonBox;
  private Joystick buttonBox_moreButtons;
  private XboxController driverXbox;
  private Joystick rightjoystick;
  private Joystick leftjoystick;
  
  // Defining commands (NOT dependent on Xbox/Joystick drive)
  private final DriveToPointCommand driveTopLeft = new DriveToPointCommand(TargetPoints.TOP_LEFT);
  private final DriveToPointCommand driveTopRight = new DriveToPointCommand(TargetPoints.TOP_RIGHT);
  private final DriveToPointCommand driveRight = new DriveToPointCommand(TargetPoints.RIGHT);
  private final DriveToPointCommand driveBottomRight = new DriveToPointCommand(TargetPoints.BOTTOM_RIGHT);
  private final DriveToPointCommand driveBottomLeft = new DriveToPointCommand(TargetPoints.BOTTOM_LEFT);
  private final DriveToPointCommand driveLeft = new DriveToPointCommand(TargetPoints.LEFT);

  private final DriveToPointCommand topStation = new DriveToPointCommand(TargetPoints.TOP_STATION);
  private final DriveToPointCommand bottomStation = new DriveToPointCommand(TargetPoints.BOTTOM_STATION);

  // Commands
  private final MoveArm armLeft = new MoveArm(arm,-0.1);
  private final MoveArm armRight = new MoveArm(arm,0.1);

  private final IntakeCommand intake = new IntakeCommand(arm, -0.15);
  private final OuttakeCommand outtake = new OuttakeCommand(arm, 0.15);

  private final MoveElevator upElevator = new MoveElevator(elevator, 0.1);
  private final MoveElevator downElevator = new MoveElevator(elevator, -0.1);

  private final ArmSetpoint armLeft65 = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_65);
  private final ArmSetpoint armLeft90 = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_90);
  private final ArmSetpoint armLeftIntake = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_INTAKE);
  private final ArmSetpoint armRight65 = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_RIGHT_65);
  private final ArmSetpoint armRight90 = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_RIGHT_90);
  private final ArmSetpoint armRightIntake = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_RIGHT_INTAKE);
  
  private final ArmSteady armSteady = new ArmSteady(arm);
  // Misc/Auto
  private final SendableChooser<Command> auto;


  public RobotContainer() {
    //Create some Subsystems
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    
    //Set Controller Ids
    if (Constants.OperatorConstants.XBOX_DRIVE) {
      driverXbox = new XboxController(0);
      buttonBox = new Joystick(1);
      buttonBox_moreButtons = new Joystick(2);
    } else {
      rightjoystick = new Joystick(0);
      leftjoystick = new Joystick(1);
      buttonBox = new Joystick(2);
      buttonBox_moreButtons = new Joystick(3);
    }

    // Configure DriveCommand
    Command driveCommand = null;
    if (OperatorConstants.XBOX_DRIVE) {
      driveCommand = drivebase.driveCommand(
          () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
          () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));
    } else {
      driveCommand = drivebase.driveCommand(
          () -> MathUtil.applyDeadband(leftjoystick.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(leftjoystick.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
          () -> -rightjoystick.getRawAxis(0));
    }
    configureBindings();

    SmartDashboard.putData(CommandScheduler.getInstance());

    eagleye.setDefaultCommand(eagleeyecommand);
    drivebase.setDefaultCommand(driveCommand);
    arm.setDefaultCommand(armSteady);

    auto = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("chooseAuto", auto);
  }


  private void configureBindings() {
    // Setpoint Commands
    new JoystickButton(buttonBox, 1).onTrue(driveTopRight);
    new JoystickButton(buttonBox, 2).onTrue(driveRight);
    new JoystickButton(buttonBox, 3).onTrue(driveBottomRight);
    new JoystickButton(buttonBox, 4).onTrue(driveBottomLeft);
    new JoystickButton(buttonBox, 5).onTrue(driveLeft);
    new JoystickButton(buttonBox, 6).onTrue(driveTopLeft);

    new JoystickButton(buttonBox, 9).onTrue(topStation); 
    new JoystickButton(buttonBox_moreButtons, 3).onTrue(bottomStation);

    // Buttonbox Arm and Elevator Commands
    new JoystickButton(buttonBox, 7).whileTrue(armLeft);
    new JoystickButton(buttonBox, 8).whileTrue(armRight);

    new JoystickButton(buttonBox, 10).whileTrue(upElevator);
    new JoystickButton(buttonBox, 11).whileTrue(downElevator);
    
    new JoystickButton(buttonBox_moreButtons, 1).whileTrue(intake);
    new JoystickButton(buttonBox_moreButtons, 2).whileTrue(outtake);
    
    //new JoystickButton(buttonBox_moreButtons, 200).onTrue(armLeft65); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 201).onTrue(armRight65); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 202).onTrue(armLeft90); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 203).onTrue(armRight90); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 204).onTrue(armLeftIntake); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 205).onTrue(armRightIntake); /* Fake ID */

    
    //new JoystickButton(buttonBox_moreButtons, 200).onTrue(armLeft65); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 201).onTrue(armRight65); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 202).onTrue(armLeft90); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 203).onTrue(armRight90); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 204).onTrue(armLeftIntake); /* Fake ID */
    //new JoystickButton(buttonBox_moreButtons, 205).onTrue(armRightIntake); /* Fake ID */

    

    if (OperatorConstants.XBOX_DRIVE) {
      new JoystickButton(driverXbox, 8).onTrue((new InstantCommand(drivebase::zeroGyro))); // (Start)
      //new JoystickButton(rightjoystick, 2).onTrue(new InstantCommand(drivebase::lock));

      // Drive Commands
      new JoystickButton(driverXbox, 2).onTrue(new DriveLocalCommandAbsolute(drivebase, 8.47, new NearestTag(drivebase, true).getTarget())); // TEST RIGHT (B)
      new JoystickButton(driverXbox, 1).onTrue(new DriveLocalCommandAbsolute(drivebase, -4.47, new NearestTag(drivebase, true).getTarget())); // TEST RIGHT (A) (6.47 ORIGINAL VAL)
      
      // Enable Drive To Nearest Target (for Matt)
      if(Constants.OperatorConstants.MATT_MODE){
        new JoystickButton(driverXbox, 4).onTrue(new NearestTag(drivebase, false)); // TEST RIGHT (Y)
      }

      // Cancel All Command
      new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(() -> {
        CommandScheduler.getInstance().cancelAll();
      })); // (X)

    }
    else{
      new JoystickButton(buttonBox_moreButtons, 3).onTrue((new InstantCommand(drivebase::zeroGyro))); // (Button 3)
      //new JoystickButton(rightjoystick, 2).onTrue(new InstantCommand(drivebase::lock));

      // Drive Commands
      new JoystickButton(leftjoystick, 5).onTrue(new DriveLocalCommandAbsolute(drivebase, 8.47, new NearestTag(drivebase, true).getTarget())); // Left Closest Thumb Button
      new JoystickButton(rightjoystick, 6).onTrue(new DriveLocalCommandAbsolute(drivebase, -4.47, new NearestTag(drivebase, true).getTarget())); // Right Closest Thumb Button (6.47 ORIGINAL VAL)
      
      // Enable Drive To Nearest Target (for Matt)
      if(Constants.OperatorConstants.MATT_MODE){
        new JoystickButton(leftjoystick, 1).onTrue(new NearestTag(drivebase, false)); // Left Trigger
      }

      new JoystickButton(rightjoystick, 2).onTrue(new InstantCommand(() -> { // Right Thumb Button
        CommandScheduler.getInstance().cancelAll();
      })); // (X)

    }
  }

  public Command getAutonomousCommand() {
    return auto.getSelected();
  }
}