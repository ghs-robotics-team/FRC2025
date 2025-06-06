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
import frc.robot.commands.NearestTag;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ArmSteady;
import frc.robot.commands.ElevatorSteady;
import frc.robot.subsystems.Climber;
import frc.robot.commands.MoveClimber;

public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem drivebase;
  private final Arm arm = new Arm(); // ALWAYS SET FACING STRAIGHT UP FOR CORRECT ENCODER POS
  private final Elevator elevator = new Elevator(); // ALWAYS SET AT BOTTOM FOR CORRECT ENCODER POS
  private final EagleEye eagleye = new EagleEye();
  private final EagleEyeCommand eagleeyecommand = new EagleEyeCommand(eagleye);
  private final Climber climber = new Climber();

  // Controllers
  private XboxController buttonsXbox;
  private XboxController driverXbox;
  private Joystick rightjoystick;
  private Joystick leftjoystick;

  // Defining Teleop commands

  // General Commands
  private final MoveArm armLeft = new MoveArm(arm,0.1);
  private final MoveArm armRight = new MoveArm(arm,-0.1);

  private final IntakeCommand intake = new IntakeCommand(arm, -1);
  private final OuttakeCommand outtake = new OuttakeCommand(arm, 1);

  private final MoveElevator upElevator = new MoveElevator(elevator, 0.13);
  private final MoveElevator downElevator = new MoveElevator(elevator, -0.13);

  private final MoveClimber upClimber = new MoveClimber(climber, 0.75);
  private final MoveClimber downClimber = new MoveClimber(climber, -0.75);

  private final ElevatorSetpoint elevatorIntake = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE); // or 0.3
  private final ArmSetpoint armHome = new ArmSetpoint(arm, 0);
  
  private final ArmSteady armSteady = new ArmSteady(arm);
  private final ElevatorSteady elevatorSteady = new ElevatorSteady(elevator);

  // Naming Convention: subsystem-Action-Level-(Order)
  // Left Top Combine Command
  private final ArmSetpoint armHomeTopOne = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armPassTop = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint armHomeTopTwo = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armPlaceTop = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_HIGH);
  private final ElevatorSetpoint elevatorPlaceTop = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_HIGH);
  private final ElevatorSetpoint elevatorReleaseTop = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_HIGH);
  private final ElevatorSetpoint elevatorZeroTop = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);

  // Left Mid Combine Command
  private final ArmSetpoint armHomeMidOne = new ArmSetpoint(arm, 0);  
  private final ArmSetpoint armPassMid = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint armHomeMidTwo = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armPlaceMid = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_MIDDLE);
  private final ElevatorSetpoint elevatorPlaceMid = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_MIDDLE);
  private final ElevatorSetpoint elevatorReleaseMid = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_MIDDLE);
  private final ElevatorSetpoint elevatorZeroMid = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);

  // Left Low Combine Command
  private final ArmSetpoint armHomeLowOne = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armPassLow = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint armHomeLowTwo = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armPlaceLow = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_LOW);
  private final ElevatorSetpoint elevatorPlaceLow = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_LOW);
  private final ElevatorSetpoint elevatorReleaseLow = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_LOW);
  private final ElevatorSetpoint elevatorZeroLow = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);

  // Left Trough Combine Command
  private final ArmSetpoint armHomeTroughOne = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armPassTrough = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint armHomeTroughTwo = new ArmSetpoint(arm, 0);
  private final ArmSetpoint armPlaceTrough = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_TROUGH);
  private final ElevatorSetpoint elevatorPlaceTrough = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_TROUGH);
  private final ElevatorSetpoint elevatorZeroTrough = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);
  private final OuttakeCommand outtakeTrough = new OuttakeCommand(arm, Constants.SetPointConstants.ELEVATOR_INTAKE);

  //  Auto Chooser
  //private final SendableChooser<Command> auto;

  // Naming Convention: (auto)-subsystem-Action-Level-(Order)
  // Auto Intake Command
  private final ArmSetpoint autoArmHomeIntakeOne = new ArmSetpoint(arm, 0);
  private final ElevatorSetpoint autoElevatorSetIntake = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);
  private final IntakeCommand autoIntakeSet = new IntakeCommand(arm, -0.5);
  private final ArmSetpoint autoArmHomeIntakeTwo = new ArmSetpoint(arm, 0);

  // Auto Left Top Combine Command
  private final ArmSetpoint autoArmHomeTopOne = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmPassTop = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint autoArmHomeTopTwo = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmPlaceTop = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_HIGH);
  private final ElevatorSetpoint autoElevatorPlaceTop = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_HIGH);
  private final ElevatorSetpoint autoElevatorZeroTop = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);

  // Auto Left Mid Combine Command
  private final ArmSetpoint autoArmHomeMidOne = new ArmSetpoint(arm, 0);  
  private final ArmSetpoint autoArmPassMid = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint autoArmHomeMidTwo = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmPlaceMid = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_MIDDLE);
  private final ElevatorSetpoint autoElevatorPlaceMid = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_MIDDLE);
  private final ElevatorSetpoint autoElevatorZeroMid = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);

  // Auto Left Low Combine Command
  private final ArmSetpoint autoArmHomeLowOne = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmPassLow = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint autoArmHomeLowTwo = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmPlaceLow = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_LOW);
  private final ElevatorSetpoint autoElevatorPlaceLow = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_LOW);
  private final ElevatorSetpoint autoElevatorZeroLow = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);

  // Auto Left Trough Combine Command
  private final ArmSetpoint autoArmHomeTroughOne = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoArmPassTrough = new ArmSetpoint(arm, -1000);
  private final ArmSetpoint autoArmHomeTroughTwo = new ArmSetpoint(arm, 0);
  private final ArmSetpoint autoarmPlaceTrough = new ArmSetpoint(arm, Constants.SetPointConstants.ARM_LEFT_TROUGH);
  private final ElevatorSetpoint autoElevatorPlaceTrough = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_TROUGH);
  private final ElevatorSetpoint autoElevatorZeroTrough = new ElevatorSetpoint(elevator, Constants.SetPointConstants.ELEVATOR_INTAKE);

  public RobotContainer() {
    // Create some Subsystems
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

    // Named Commands

    // Auto Intake
    NamedCommands.registerCommand("Intake",
      autoArmHomeIntakeOne.withTimeout(1).andThen(
        autoElevatorSetIntake.withTimeout(2)).andThen(
        autoIntakeSet.withTimeout(4)).andThen( //Change Time to limit switch?
        autoArmHomeIntakeTwo.withTimeout(1)) 
    );

    // Auto Dislodge
    NamedCommands.registerCommand("Dislodge", new MoveElevator(elevator, 0.73).withTimeout(0.2).andThen(new WaitCommand(0.1))
    .andThen(new MoveElevator(elevator, -0.73).withTimeout(0.2)));

    // Auto Left Top Place
    NamedCommands.registerCommand("Left Top Place",
      autoArmHomeTopOne.withTimeout(1).andThen(
      autoElevatorPlaceTop.withTimeout(2)).andThen(
      autoArmPlaceTop.withTimeout(1))
    );

    NamedCommands.registerCommand("Left Top Place Parallel",
      autoElevatorZeroTop.withTimeout(2).alongWith(new WaitCommand(0.13).andThen(autoArmPassTop.withTimeout(1))).andThen(
    autoArmHomeTopTwo.withTimeout(1))
    );

    // Set Controller Ids
    if (Constants.OperatorConstants.XBOX_DRIVE) {
      driverXbox = new XboxController(0);
      buttonsXbox = new XboxController(1);
      Globals.buttonsXbox = buttonsXbox;
    } else {
      rightjoystick = new Joystick(0);
      leftjoystick = new Joystick(1);
      buttonsXbox = new XboxController(2);
      Globals.buttonsXbox = buttonsXbox;
    }

    // Configure DriveCommand
    Command driveCommand = null;
    if (OperatorConstants.XBOX_DRIVE) {
      driveCommand = drivebase.driveCommand(
          () -> MathUtil.applyDeadband(-driverXbox.getLeftY()*Globals.inversion, OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(-driverXbox.getLeftX()*Globals.inversion, OperatorConstants.LEFT_X_DEADBAND),
          () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));
    } else {
      driveCommand = drivebase.driveCommand(
          () -> MathUtil.applyDeadband(leftjoystick.getRawAxis(1)*Globals.inversion, OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(leftjoystick.getRawAxis(0)*Globals.inversion, OperatorConstants.LEFT_X_DEADBAND),
          () -> -rightjoystick.getRawAxis(0));
    }
    configureBindings();

    SmartDashboard.putData(CommandScheduler.getInstance());

    eagleye.setDefaultCommand(eagleeyecommand);
    drivebase.setDefaultCommand(driveCommand);
    arm.setDefaultCommand(armSteady);
    elevator.setDefaultCommand(elevatorSteady);

    //auto = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("chooseAuto", auto); 
  }

  private void configureBindings() {
    // ButtonXbox Arm and Elevator Commands
    new POVButton(buttonsXbox, 90).whileTrue(armRight);
    new POVButton(buttonsXbox, 270).whileTrue(armLeft);

    new POVButton(buttonsXbox, 0).whileTrue(upElevator);
    new POVButton(buttonsXbox, 180).whileTrue(downElevator);

    //new JoystickButton(buttonsXbox, 8).onTrue(elevatorZero); // Uneeded for Now
    new JoystickButton(buttonsXbox, 10).onTrue(elevatorIntake);

    new JoystickButton(buttonsXbox, 7).whileTrue(upClimber);
    new JoystickButton(buttonsXbox, 8).whileTrue(downClimber);

    new JoystickButton(buttonsXbox, 6).whileTrue(intake);
    new JoystickButton(buttonsXbox, 5).whileTrue(outtake);
    
    new JoystickButton(buttonsXbox, 9).onTrue(armHome);
    

    //Place Left High
    new JoystickButton(buttonsXbox, 4).onTrue( // Y
      armHomeTopOne.andThen(
      elevatorPlaceTop)
      );
    new JoystickButton(buttonsXbox, 4).onFalse( // Y
      elevatorReleaseTop.andThen(
      armPlaceTop).andThen(
      elevatorZeroTop.alongWith(new WaitCommand(0.13).andThen(
      armHomeTopTwo))) 
      );
      
    //Place Left Middle
    new JoystickButton(buttonsXbox, 2).onTrue( // B
      armHomeMidOne.andThen(
      elevatorPlaceMid)
      );
    new JoystickButton(buttonsXbox, 2).onFalse( // B
      elevatorReleaseMid.andThen(
      armPlaceMid).andThen(
      elevatorZeroMid.alongWith(new WaitCommand(0.13))).andThen(
      armHomeMidTwo) 
      );

    /*//Place Left Low
    new JoystickButton(buttonsXbox, 1).onTrue( // A
      armHomeLowOne.andThen(
      elevatorPlaceLow)
      );
    new JoystickButton(buttonsXbox, 1).onFalse( // A
      elevatorReleaseLow.andThen(
      armPlaceLow).andThen(
      elevatorZeroLow.alongWith(new WaitCommand(0.13))).andThen(
      armHomeLowTwo)
      ); */

    //Place Left Trough
    new JoystickButton(buttonsXbox, 3).onTrue( // X
      armHomeTroughOne.andThen(
      elevatorPlaceTrough).andThen(
      armPlaceTrough.withTimeout(0.5)).andThen(
        new WaitCommand(0.1)).andThen(
        outtakeTrough.withTimeout(0.7)).andThen(new WaitCommand(0.5)).andThen(
        elevatorZeroTrough.alongWith(armPassTrough)).andThen(
      armHomeTroughTwo) 
      );
    
    if (OperatorConstants.XBOX_DRIVE) { // Untested
      new JoystickButton(driverXbox, 8).onTrue((new InstantCommand(drivebase::zeroGyro))); // (Start)
      //new JoystickButton(rightjoystick, 2).onTrue(new InstantCommand(drivebase::lock));

      // Enable Drive To Nearest Target (for Matt)
      if(Constants.OperatorConstants.MATT_MODE){
        new JoystickButton(driverXbox, 2).onTrue(new NearestTag(drivebase, false, -2, driverXbox)); // (B)
        new JoystickButton(driverXbox, 3).onTrue(new NearestTag(drivebase, false, -15, driverXbox)); // (x)
      }

      // Cancel All Command
      new JoystickButton(driverXbox, 4).onTrue(new InstantCommand(() -> { // (Y)
        CommandScheduler.getInstance().cancelAll();
      })); 
    }
    else{
      new JoystickButton(leftjoystick, 4).onTrue((new InstantCommand(drivebase::zeroGyro))); // (Button 4) (Left Thumb Button)
      //new JoystickButton(rightjoystick, 2).onTrue(new InstantCommand(drivebase::lock));

      // Enable Drive To Nearest Target (for Matt)
      /*if(Constants.OperatorConstants.MATT_MODE){
        new JoystickButton(leftjoystick, 12).onTrue(new NearestTag(drivebase, false, -2, buttonsXbox)); 
        new JoystickButton(leftjoystick, 11).onTrue(new NearestTag(drivebase, false, -15, buttonsXbox));
      }*/

      new JoystickButton(rightjoystick, 3).onTrue(new InstantCommand(() -> { // Right Thumb Button
        CommandScheduler.getInstance().cancelAll();
      })); 
    }
  }

  public Command getAutonomousCommand() {
    //return auto.getSelected();
    return null;
  }
}