package frc.robot;

import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.TargetPoints;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.EagleEyeCommand;
import frc.robot.commands.ElevatorSetpoint;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EagleEye;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.MoveArm;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.ArmSetpoint;
import frc.robot.commands.DriveLocalCommandAbsolute;
import frc.robot.commands.NearestTag;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ArmSteady;
import frc.robot.commands.ElevatorSteady;
import frc.robot.subsystems.Climber;
import frc.robot.commands.MoveClimber;


public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem drivebase;
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator(); // ALWAYS SET AT BOTTOM FOR CORRECT ENCODER POS
  private final EagleEye eagleye = new EagleEye();
  private final EagleEyeCommand eagleeyecommand = new EagleEyeCommand(eagleye);
  private final Climber climber = new Climber();

  // Controllers
  private Joystick buttonBox;
  private Joystick buttonBox_moreButtons;
  private XboxController buttonsXbox;
  private XboxController driverXbox;
  private Joystick rightjoystick;
  private Joystick leftjoystick;
  
  // Defining Teleop commands

  // Drive Setpoints
  private final DriveToPointCommand driveTopLeft = new DriveToPointCommand(TargetPoints.TOP_LEFT);
  private final DriveToPointCommand driveTopRight = new DriveToPointCommand(TargetPoints.TOP_RIGHT);
  private final DriveToPointCommand driveRight = new DriveToPointCommand(TargetPoints.RIGHT);
  private final DriveToPointCommand driveBottomRight = new DriveToPointCommand(TargetPoints.BOTTOM_RIGHT);
  private final DriveToPointCommand driveBottomLeft = new DriveToPointCommand(TargetPoints.BOTTOM_LEFT);
  private final DriveToPointCommand driveLeft = new DriveToPointCommand(TargetPoints.LEFT);

  private final DriveToPointCommand topStation = new DriveToPointCommand(TargetPoints.TOP_STATION);
  private final DriveToPointCommand bottomStation = new DriveToPointCommand(TargetPoints.BOTTOM_STATION);

  // General Commands
  private final MoveArm armLeft = new MoveArm(arm,0.1);
  private final MoveArm armRight = new MoveArm(arm,-0.1);

  private final IntakeCommand intake = new IntakeCommand(arm, -0.5);
  private final OuttakeCommand outtake = new OuttakeCommand(arm, 0.5);

  private final MoveElevator upElevator = new MoveElevator(elevator, 0.1);
  private final MoveElevator downElevator = new MoveElevator(elevator, -0.1);

  private final MoveClimber upClimber = new MoveClimber(climber, 0.75);
  private final MoveClimber downClimber = new MoveClimber(climber, -0.75);

  private final ElevatorSetpoint elevatorZero = new ElevatorSetpoint(elevator, 0.3);
  private final ElevatorSetpoint elevatorIntake = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);
  private final ElevatorSetpoint elevatorLow = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_LOW);
  private final ElevatorSetpoint elevatorMiddle = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_MIDDLE);
  private final ElevatorSetpoint elevatorHigh = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_HIGH);
  private final ElevatorSetpoint elevatorTrough = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_TROUGH);

  private final ArmSetpoint armLeftLow = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_LOW);
  private final ArmSetpoint armLeftMiddle = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_MIDDLE);
  private final ArmSetpoint armLeftHigh = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_HIGH);
  private final ArmSetpoint armLeftTrough = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_TROUGH);
  private final ArmSetpoint armRightLow = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_RIGHT_LOW);
  private final ArmSetpoint armRightMiddle = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_RIGHT_MIDDLE);
  private final ArmSetpoint armRightHigh = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_RIGHT_HIGH);
  private final ArmSetpoint armRightTrough = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_RIGHT_TROUGH);
  private final ArmSetpoint armHome = new ArmSetpoint(arm, 0);
  
  private final ArmSteady armSteady = new ArmSteady(arm);
  private final ElevatorSteady elevatorSteady = new ElevatorSteady(elevator);

  // Intake Command
  private final ArmSetpoint armHomeIntake = new ArmSetpoint(arm, 0);
  private final ElevatorSetpoint elevatorSetIntake = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);
  private final IntakeCommand intakeSet = new IntakeCommand(arm, -0.5);
  private final ArmSetpoint armHomeIntakeFinal = new ArmSetpoint(arm, 0);

  // Left Top Combine Command
  private final ArmSetpoint armHomeLeftTop = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armRightLeftTopFinal = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint armHomeLeftTopFinal = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armLeftHighLeftTop = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_HIGH);
  private final ElevatorSetpoint elevatorHighLeftTop = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_HIGH);
  private final ElevatorSetpoint elevatorZeroLeftTop = new ElevatorSetpoint(elevator, 0.3);

  // Left Mid Combine Command
  private final ArmSetpoint armHomeLeftMid = new ArmSetpoint(arm, 0);  
  private final ArmSetpoint armRightLeftMidFinal = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint armHomeLeftMidFinal = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armLeftHighLeftMid = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_MIDDLE);
  private final ElevatorSetpoint elevatorHighLeftMid = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_MIDDLE);
  private final ElevatorSetpoint elevatorZeroLeftMid = new ElevatorSetpoint(elevator, 0.3);

  // Left Low Combine Command
  private final ArmSetpoint armHomeLeftLow = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armRightLeftLowFinal = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint armHomeLeftLowFinal = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armLeftHighLeftLow = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_LOW);
  private final ElevatorSetpoint elevatorHighLeftLow = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_LOW);
  private final ElevatorSetpoint elevatorZeroLeftLow = new ElevatorSetpoint(elevator, 0.3);

  // Left Low Combine Command
  private final ArmSetpoint armHomeLeftTrough = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armRightLeftTroughFinal = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint armHomeLeftTroughFinal = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armLeftHighLeftTrough = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_TROUGH);
  private final ElevatorSetpoint elevatorHighLeftTrough = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_TROUGH);
  private final ElevatorSetpoint elevatorZeroLeftTrough = new ElevatorSetpoint(elevator, 0.3);

  //  Auto Chooser
  private final SendableChooser<Command> auto;

  // Auto Named Commands

  // Auto Intake Command
  private final ArmSetpoint autoArmHomeIntake = new ArmSetpoint(arm, 0);
  private final ElevatorSetpoint autoElevatorSetIntake = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);
  private final IntakeCommand autoIntakeSet = new IntakeCommand(arm, -0.5);
  private final ArmSetpoint autoArmHomeIntakeFinal = new ArmSetpoint(arm, 0);

  // Auto Left Top Combine Command
  private final ArmSetpoint autoArmHomeLeftTop = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmRightLeftTopFinal = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint autoArmHomeLeftTopFinal = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmLeftHighLeftTop = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_HIGH);
  private final ElevatorSetpoint autoElevatorHighLeftTop = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_HIGH);
  private final ElevatorSetpoint autoElevatorZeroLeftTop = new ElevatorSetpoint(elevator, 0.3);

  // Auto Left Mid Combine Command
  private final ArmSetpoint autoArmHomeLeftMid = new ArmSetpoint(arm, 0);  
  private final ArmSetpoint autoArmRightLeftMidFinal = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint autoArmHomeLeftMidFinal = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmLeftHighLeftMid = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_MIDDLE);
  private final ElevatorSetpoint autoElevatorHighLeftMid = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_MIDDLE);
  private final ElevatorSetpoint autoElevatorZeroLeftMid = new ElevatorSetpoint(elevator, 0.3);

  // Auto Left Low Combine Command
  private final ArmSetpoint autoArmHomeLeftLow = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmRightLeftLowFinal = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint autoArmHomeLeftLowFinal = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmLeftHighLeftLow = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_LOW);
  private final ElevatorSetpoint autoElevatorHighLeftLow = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_LOW);
  private final ElevatorSetpoint autoElevatorZeroLeftLow = new ElevatorSetpoint(elevator, 0.3);

  // Auto Left Low Combine Command
  private final ArmSetpoint autoArmHomeLeftTrough = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmRightLeftTroughFinal = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint autoArmHomeLeftTroughFinal = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmLeftHighLeftTrough = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_TROUGH);
  private final ElevatorSetpoint autoElevatorHighLeftTrough = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_TROUGH);
  private final ElevatorSetpoint autoElevatorZeroLeftTrough = new ElevatorSetpoint(elevator, 0.3);


  public RobotContainer() {
    // Create some Subsystems
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    
    // Set Controller Ids
    if (Constants.OperatorConstants.XBOX_DRIVE) {
      driverXbox = new XboxController(0);
      buttonsXbox = new XboxController(1);
      buttonBox = new Joystick(2);
      buttonBox_moreButtons = new Joystick(3);
    } else {
      rightjoystick = new Joystick(0);
      leftjoystick = new Joystick(1);
      buttonsXbox = new XboxController(2);
      buttonBox = new Joystick(3);
      buttonBox_moreButtons = new Joystick(4);
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
    //arm.setDefaultCommand(armSteady);
    //elevator.setDefaultCommand(elevatorSteady);

    auto = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("chooseAuto", auto); 


    // Named Commands

    // Auto Intake
    NamedCommands.registerCommand("Intake",
      autoArmHomeIntake.andThen(
        autoElevatorSetIntake).andThen(
        autoIntakeSet.withTimeout(3)).andThen( //Change Time to limit switch?
        autoArmHomeIntakeFinal) 
    );

    // Auto Left Top Place
    NamedCommands.registerCommand("Left Top Place",
      autoArmHomeLeftTop.andThen(
      autoElevatorHighLeftTop).andThen(
      autoArmLeftHighLeftTop).andThen(
        autoElevatorZeroLeftTop.alongWith(new WaitCommand(0.5).andThen(autoArmRightLeftTopFinal))).andThen(
      autoArmHomeLeftTopFinal) 
    );

    // Auto Left Middle Place
    NamedCommands.registerCommand("Left Middle Place",
      autoArmHomeLeftMid.andThen(
      autoElevatorHighLeftMid).andThen(
      autoArmLeftHighLeftMid).andThen(
        autoElevatorZeroLeftMid.alongWith(new WaitCommand(0.5).andThen(autoArmRightLeftMidFinal))).andThen(
      autoArmHomeLeftMidFinal) 
    );

    // Auto Left Low Place
    NamedCommands.registerCommand("Left Low Place",
      autoArmHomeLeftLow.andThen(
      autoElevatorHighLeftLow).andThen(
      autoArmLeftHighLeftLow).andThen(
        autoElevatorZeroLeftLow.alongWith(new WaitCommand(0.5).andThen(autoArmRightLeftLowFinal))).andThen(
      autoArmHomeLeftLowFinal) 
    );

    // Auto Left Trough Place
    NamedCommands.registerCommand("Left Low Place",
      autoArmHomeLeftTrough.andThen(
      autoElevatorHighLeftTrough).andThen(
      autoArmLeftHighLeftTrough).andThen(
        autoElevatorZeroLeftTrough.alongWith(new WaitCommand(0.5).andThen(autoArmRightLeftTroughFinal))).andThen(
      autoArmHomeLeftTroughFinal) 
    );
  }


  private void configureBindings() {
    // Setpoint Commands
    /*new JoystickButton(buttonBox, 1).onTrue(driveTopRight);
    new JoystickButton(buttonBox, 2).onTrue(driveRight);
    new JoystickButton(buttonBox, 3).onTrue(driveBottomRight);
    new JoystickButton(buttonBox, 4).onTrue(driveBottomLeft);
    new JoystickButton(buttonBox, 5).onTrue(driveLeft);
    new JoystickButton(buttonBox, 6).onTrue(driveTopLeft);

    new JoystickButton(buttonBox, 9).onTrue(topStation); 
    new JoystickButton(buttonBox_moreButtons, 3).onTrue(bottomStation); */

    // Buttonbox Arm and Elevator Commands
    //new JoystickButton(buttonBox, 5).whileTrue(armLeft); 
    //new JoystickButton(buttonBox, 2).whileTrue(armRight);
    
    new POVButton(buttonsXbox, 90).whileTrue(armRight);
    new POVButton(buttonsXbox, 270).whileTrue(armLeft);

    //new JoystickButton(buttonBox, 1).whileTrue(upElevator);
    //new JoystickButton(buttonBox, 6).whileTrue(downElevator);

    new POVButton(buttonsXbox, 0).whileTrue(upElevator);
    new POVButton(buttonsXbox, 180).whileTrue(downElevator);

    //new JoystickButton(buttonBox, 4).onTrue(elevatorZero);
    new JoystickButton(buttonsXbox, 8).onTrue(elevatorZero);
    //new JoystickButton(buttonBox, 3).onTrue(elevatorIntake);
    //new JoystickButton(buttonBox_moreButtons, 3).onTrue(elevatorTrough);    
    //new JoystickButton(buttonBox, 9).onTrue(elevatorHigh); 
    //new JoystickButton(buttonBox, 10).onTrue(elevatorMiddle); 
    //new JoystickButton(buttonBox, 11).onTrue(elevatorLow);

    //new JoystickButton(buttonBox_moreButtons, 1).whileTrue(intake);
    //new JoystickButton(buttonBox_moreButtons, 2).whileTrue(outtake);

    new JoystickButton(buttonsXbox, 6).whileTrue(intake);
    new JoystickButton(buttonsXbox, 5).whileTrue(outtake);
    
    //new JoystickButton(buttonBox, 7).onTrue(armLeftMiddle); 
    //new JoystickButton(buttonBox, 8).onTrue(armRightMiddle);
    //new JoystickButton(buttonBox, 12).onTrue(armHome);
    new JoystickButton(buttonsXbox, 7).onTrue(armHome);
    
    //Place Left High
    new JoystickButton(buttonsXbox, 3).onTrue(
      armHomeLeftTop.andThen(
      elevatorHighLeftTop).andThen(
      armLeftHighLeftTop).andThen(
        elevatorZeroLeftTop.alongWith(new WaitCommand(0.5).andThen(armRightLeftTopFinal))).andThen(
      armHomeLeftTopFinal) 
      ); 

    //Place Left Middle
    new JoystickButton(buttonsXbox, 2).onTrue(
      armHomeLeftMid.andThen(
      elevatorHighLeftMid).andThen(
      armLeftHighLeftMid).andThen(
        elevatorZeroLeftMid.alongWith(new WaitCommand(0.5).andThen(armRightLeftMidFinal))).andThen(
      armHomeLeftMidFinal) 
      );

    //Place Left Low
    new JoystickButton(buttonsXbox, 1).onTrue(
      armHomeLeftLow.andThen(
      elevatorHighLeftLow).andThen(
      armLeftHighLeftLow).andThen(
        elevatorZeroLeftLow.alongWith(new WaitCommand(0.5).andThen(armRightLeftLowFinal))).andThen(
      armHomeLeftLowFinal) 
      );

    //Place Left Trough
    new JoystickButton(buttonsXbox, 3).onTrue(
      armHomeLeftTrough.andThen(
      elevatorHighLeftTrough).andThen(
      armLeftHighLeftTrough).andThen(
        elevatorZeroLeftTrough.alongWith(new WaitCommand(0.5).andThen(armRightLeftTroughFinal))).andThen(
      armHomeLeftTroughFinal) 
      );

    
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

      //new JoystickButton(driverXbox, 6).whileTrue(upClimber);// Right High Trigger
      //new JoystickButton(driverXbox, 5).whileTrue(downClimber);// Left High Trigger

    }
    else{
      new JoystickButton(rightjoystick, 3).onTrue((new InstantCommand(drivebase::zeroGyro))); // (Button 3) (Left Thumb Button)
      //new JoystickButton(rightjoystick, 2).onTrue(new InstantCommand(drivebase::lock));

      // Drive Commands
      //new JoystickButton(leftjoystick, 5).onTrue(new DriveLocalCommandAbsolute(drivebase, 8.47, new NearestTag(drivebase, true).getTarget())); // Left Closest Thumb Button
      //new JoystickButton(rightjoystick, 6).onTrue(new DriveLocalCommandAbsolute(drivebase, -4.47, new NearestTag(drivebase, true).getTarget())); // Right Closest Thumb Button (6.47 ORIGINAL VAL)
      
      // Enable Drive To Nearest Target (for Matt)
      if(Constants.OperatorConstants.MATT_MODE){
        //new JoystickButton(leftjoystick, 1).onTrue(new NearestTag(drivebase, false)); // Left Trigger
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