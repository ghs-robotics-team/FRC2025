// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NearestTag extends Command {
  /** Creates a new NearestTag. */
  SwerveSubsystem swerve;
  DriveToPointCommandForPose2d command;
  boolean stat;
  public NearestTag(SwerveSubsystem swerve, boolean stat) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    this.swerve = swerve;
    this.command = null;
    this.stat = stat;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Pose2d> sides = new ArrayList<Pose2d>();
    for (TargetPoints point : TargetPoints.values()){
      sides.add(point.get());
    }

    Pose2d target = sides.get(0);
    for (Pose2d side : sides){
      if(swerve.getPose().getTranslation().getDistance(side.getTranslation()) < swerve.getPose().getTranslation().getDistance(target.getTranslation())){
        target = side;
      }
    }

    command = new DriveToPointCommandForPose2d(target);
    if(!stat){
      command.schedule();
    }
}

public Pose2d nearTag(){
  List<Pose2d> sides = new ArrayList<Pose2d>();
  for (TargetPoints point : TargetPoints.values()){
    sides.add(point.get());
  }

  Pose2d target = sides.get(0);
  for (Pose2d side : sides){
    if(swerve.getPose().getTranslation().getDistance(side.getTranslation()) < swerve.getPose().getTranslation().getDistance(target.getTranslation())){
      target = side;
    }
  }
  SmartDashboard.putNumber("X", target.getX());
  SmartDashboard.putNumber("Y", target.getY());
  SmartDashboard.putNumber("bX", TargetPoints.LEFT.get().getX());
  SmartDashboard.putNumber("bY", TargetPoints.LEFT.get().getY());
  return target;
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
    return true;
  }
}
