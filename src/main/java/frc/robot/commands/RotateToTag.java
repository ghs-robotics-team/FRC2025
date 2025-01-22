// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToTag extends Command {
  /** Creates a new RotateToTag. */
  SwerveSubsystem swerve;
  PIDController pid;
  public RotateToTag(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    this.swerve = swerve;
    this.pid = new PIDController(0.08, 0, 0.004);
    pid.enableContinuousInput(-180, 180);
    SmartDashboard.putNumber("PID-P", pid.getP());
    SmartDashboard.putNumber("PID-I", pid.getI());
    SmartDashboard.putNumber("PID-D", pid.getD());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.drive(new Translation2d(0, 0), 0, true);
   /*  LimelightTarget_Fiducial[] target_Fiducials = LimelightHelpers
        .getLatestResults("limelight-cam1").targetingResults.targets_Fiducials; */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
