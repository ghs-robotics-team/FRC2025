package frc.robot;

import java.io.File;
import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EagleEye;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ArmLeft;
import frc.robot.commands.ArmRight;
import frc.robot.commands.DriveLocalCommandAbsolute;
import frc.robot.commands.NearestTag;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase;
  private final Arm arm = new Arm();
  private final Elevator elevator = new Elevator();

  private Joystick buttonBox;
  private XboxController driverXbox;
  private Joystick rightjoystick;
  private Joystick leftjoystick;
  private final EagleEye eagleye = new EagleEye();
  private final EagleEyeCommand eagleeyecommand = new EagleEyeCommand(eagleye);

  // Defining commands (NOT dependent on Xbox/Joystick drive)
  private final DriveToPointCommand driveTopLeft = new DriveToPointCommand(TargetPoints.TOP_LEFT);
  private final DriveToPointCommand driveTopRight = new DriveToPointCommand(TargetPoints.TOP_RIGHT);
  private final DriveToPointCommand driveRight = new DriveToPointCommand(TargetPoints.RIGHT);
  private final DriveToPointCommand driveBottomRight = new DriveToPointCommand(TargetPoints.BOTTOM_RIGHT);
  private final DriveToPointCommand driveBottomLeft = new DriveToPointCommand(TargetPoints.BOTTOM_LEFT);
  private final DriveToPointCommand driveLeft = new DriveToPointCommand(TargetPoints.LEFT);

  private final ArmLeft armLeft = new ArmLeft(arm);
  private final ArmRight armRight = new ArmRight(arm);



  public RobotContainer() {
    drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
    
    eagleye.setDefaultCommand(eagleeyecommand);


    if (Constants.OperatorConstants.XBOX_DRIVE) {
      driverXbox = new XboxController(0);
      buttonBox = new Joystick(1);
    } else {
      rightjoystick = new Joystick(0);
      leftjoystick = new Joystick(1);
      buttonBox = new Joystick(2);
    }

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
    drivebase.setDefaultCommand(driveCommand);
  }


  private void configureBindings() {
    // Binding commands
    new JoystickButton(buttonBox, 1).onTrue(driveTopRight);
    new JoystickButton(buttonBox, 2).onTrue(driveRight);
    new JoystickButton(buttonBox, 3).onTrue(driveBottomRight);
    new JoystickButton(buttonBox, 4).onTrue(driveBottomLeft);
    new JoystickButton(buttonBox, 5).onTrue(driveLeft);
    new JoystickButton(buttonBox, 6).onTrue(driveTopLeft);


    if (OperatorConstants.XBOX_DRIVE) {
      new JoystickButton(driverXbox, 8).onTrue((new InstantCommand(drivebase::zeroGyro))); // (Start)

      new JoystickButton(driverXbox, 2).onTrue(new DriveLocalCommandAbsolute(drivebase, 8.47, new NearestTag(drivebase, true).getTarget())); // TEST RIGHT (B)
      new JoystickButton(driverXbox, 1).onTrue(new DriveLocalCommandAbsolute(drivebase, -4.47, new NearestTag(drivebase, true).getTarget())); // TEST RIGHT (A) (6.47 ORIGINAL VAL)
      //new JoystickButton(rightjoystick, 2).onTrue(new InstantCommand(drivebase::lock));
      if(Constants.OperatorConstants.MATT_MODE){
        new JoystickButton(driverXbox, 4).onTrue(new NearestTag(drivebase, false)); // TEST RIGHT (Y)
      }
      new JoystickButton(driverXbox, 5).onTrue(armLeft);
      new JoystickButton(driverXbox, 6).onTrue(armRight);

      
      new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(() -> {
        CommandScheduler.getInstance().cancelAll();
      })); // X
    }
  }

  public Command getAutonomousCommand() {
    return null;
  }
}