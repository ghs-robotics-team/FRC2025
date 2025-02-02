// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveLocalCommand extends Command {
  /** Creates a new RotateToTag. */
  SwerveSubsystem swerve;
  PIDController pidx;
  PIDController pidy;
  double toX;
  double toY;
  double inches;
  double xDistance;
  double yDistance;
  Pose2d newPos;

  public DriveLocalCommand(SwerveSubsystem swerve, double inches) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    this.swerve = swerve;
    /*this.pidx = new PIDController(1, 0, 0.004); // set PID directions
    this.pidy = new PIDController(1, 0, 0.004); // set PID directions
    this.inches = inches;

    SmartDashboard.putNumber("XLocal-PID-P", pidx.getP());
    SmartDashboard.putNumber("XLocal-PID-I", pidx.getI());
    SmartDashboard.putNumber("XLocal-PID-D", pidx.getD());

    SmartDashboard.putNumber("YLocal-PID-P", pidy.getP());
    SmartDashboard.putNumber("YLocal-PID-I", pidy.getI());
    SmartDashboard.putNumber("YLocal-PID-D", pidy.getD());*/
  }

  public Pose2d distanceToPos(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();

    double angle = pose.getRotation().getRadians();

    if(Math.abs(pose.getRotation().getDegrees()) > 90){
      x -= Units.inchesToMeters(inches) * Math.cos(angle);
      y -= Units.inchesToMeters(inches) * Math.sin(angle); 
    }
    else{
      x += Units.inchesToMeters(inches) * Math.cos(angle);
      y += Units.inchesToMeters(inches) * Math.sin(angle); 
    }
    

    SmartDashboard.putNumber("NewX", x);
    SmartDashboard.putNumber("NewY", y);

    return new Pose2d(x, y, pose.getRotation()); 
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.drive(new Translation2d(0, 0), 0, true); 

    /*double Px = SmartDashboard.getNumber("XLocal-PID-P", 1.0 / 150.0);
    double Ix = SmartDashboard.getNumber("XLocal-PID-I", 0.0);
    double Dx = SmartDashboard.getNumber("XLocal-PID-D", 0);
    // Set PID numbers
    pidx.setP(Px);
    pidx.setI(Ix);
    pidx.setD(Dx);

    double Py = SmartDashboard.getNumber("YLocal-PID-P", 1.0 / 150.0);
    double Iy = SmartDashboard.getNumber("YLocal-PID-I", 0.0);
    double Dy = SmartDashboard.getNumber("YLocal-PID-D", 0);
    // Set PID numbers
    pidy.setP(Py);
    pidy.setI(Iy);
    pidy.setD(Dy);

    Pose2d newPose = distanceToPos(swerve.getPose());
    xDistance = newPose.getX();
    yDistance = newPose.getY();*/
    newPos = distanceToPos(swerve.getPose());
    swerve.driveToPose(newPos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*toX = pidx.calculate(swerve.getPose().getX(), xDistance);
    toY = pidy.calculate(swerve.getPose().getY(), yDistance);

    double magnitude = Math.hypot(toX, toY);
    if(magnitude > 1){
      toX /= magnitude;
      toY /= magnitude;
    }

    double headingError = swerve.getPose().getRotation().getRadians();
    double headingCorrection = -0.02 * headingError;

    SmartDashboard.putNumber("PIDxError", pidx.getPositionError());
    SmartDashboard.putNumber("PIDyError", pidy.getPositionError());

    SmartDashboard.putNumber("toX", toX);
    SmartDashboard.putNumber("toY", toY);

    SmartDashboard.putNumber("AngleLocal", swerve.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("XPos", swerve.getPose().getX());
    SmartDashboard.putNumber("YPos", swerve.getPose().getY());


    if (pidx.getPositionError() > -Units.inchesToMeters(5) && pidx.getPositionError() < Units.inchesToMeters(5) && pidy.getPositionError() > -Units.inchesToMeters(5) && pidy.getPositionError() < Units.inchesToMeters(5)) {
      swerve.drive(new Translation2d(0, 0), 0, true);
      // deadzone
    } else {
      swerve.drive(new Translation2d(toX*swerve.getSwerveDrive().getMaximumChassisVelocity(), toY*swerve.getSwerveDrive().getMaximumChassisVelocity()), headingCorrection, true);
    }*/

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if (pidx.getPositionError() > -Units.inchesToMeters(1) && pidx.getPositionError() < Units.inchesToMeters(1) && pidy.getPositionError() > -Units.inchesToMeters(5) && pidy.getPositionError() < Units.inchesToMeters(5)) {
      swerve.drive(new Translation2d(0, 0), 0, true);
      return true;
      // deadzone
    }
    else{
      return false;
    }*/
    if(swerve.getPose().getTranslation().getDistance(newPos.getTranslation())<0.1){
      return true;
    }
    return false;
  }
}

