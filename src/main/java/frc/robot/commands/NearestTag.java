// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class NearestTag extends Command {
  /** Creates a new NearestTag. */
  SwerveSubsystem swerve;
  DriveToPointCommandForPose2d command;
  boolean stat;
  Pose2d target;

  public NearestTag(SwerveSubsystem swerve, boolean stat) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    this.swerve = swerve;
    this.command = null;
    this.stat = stat;
    this.target = swerve.getPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create list for targetpoint Pose2d Values.
    List<Pose2d> sides = new ArrayList<Pose2d>();

    // Add each target point to sides list as Pose2d.
    for (TargetPoints point : TargetPoints.values()){
      sides.add(point.get());
    }

    // Loop Finds the closest target point to the robot.
    target = sides.get(0);
    for (Pose2d side : sides){
      if(swerve.getPose().getTranslation().getDistance(side.getTranslation()) < swerve.getPose().getTranslation().getDistance(target.getTranslation())){
        target = side;
      }
    }

    // Create DriveToPointCommand for the closest target point.
    command = new DriveToPointCommandForPose2d(target);
    if(!stat){
      command.schedule();
    }
  }

  public Pose2d getTarget(){
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
