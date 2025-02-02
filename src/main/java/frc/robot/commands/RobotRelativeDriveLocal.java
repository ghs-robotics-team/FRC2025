// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RobotRelativeDriveLocal extends Command {
  /** Creates a new RotateToTag. */
  SwerveSubsystem swerve;
  PIDController pid;
  double direction;
  double measurement;
  public RobotRelativeDriveLocal(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    this.swerve = swerve;
    this.pid = new PIDController(0.08, 0, 0.004); // set PID directions
    pid.enableContinuousInput(-180, 180);
    SmartDashboard.putNumber("Local-PID-P", pid.getP());
    SmartDashboard.putNumber("Local-PID-I", pid.getI());
    SmartDashboard.putNumber("Local-PID-D", pid.getD());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.drive(new Translation2d(0, 0), 0, true);
    measurement = 0;
    double P = SmartDashboard.getNumber("Local-PID-P", 1.0 / 150.0);
    double I = SmartDashboard.getNumber("Local-PID-I", 0.0);
    double D = SmartDashboard.getNumber("Local-PID-D", 0);
    // Set PID numbers
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    direction = pid.calculate(measurement, 6.47);
    measurement += direction*0.003333;

    if (pid.getPositionError() > -0.25 && pid.getPositionError() < 0.25) {
      swerve.drive(new Translation2d(0, 0), 0, false);
      // deadzone
    } else {
      swerve.drive(new Translation2d(0, Units.inchesToMeters(direction)), 0, false);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

