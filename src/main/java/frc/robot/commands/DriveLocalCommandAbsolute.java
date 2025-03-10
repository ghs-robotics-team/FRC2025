package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveLocalCommandAbsolute extends Command {

  SwerveSubsystem swerve;
  
  PIDController pidx;
  PIDController pidy;

  Pose2d target;
  double toX;
  double toY;
  double inches;
  Pose2d newPos;
  double xError;
  double yError;

  public DriveLocalCommandAbsolute(SwerveSubsystem swerve, double inches, Pose2d target) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.pidy = new PIDController(3, 0, 0.004); // set PID directions
    this.inches = inches;
    this.target = target;

    // Put PID Values.
    SmartDashboard.putNumber("DLCA YLocal PID P", pidy.getP());
    SmartDashboard.putNumber("DLCA YLocal PID I", pidy.getI());
    SmartDashboard.putNumber("DLCA YLocal PID D", pidy.getD());
  }

  public Pose2d distanceToPos(Pose2d pose) {
    // Current X and Y Position of the Robot.
    double x = pose.getX();
    double y = pose.getY();
    double angle_degree;

    // Correcting for Weird WPILib Angle Measurements.
    if (inches >= 0) {
      angle_degree = pose.getRotation().getDegrees() - 90;
    } else {
      angle_degree = pose.getRotation().getDegrees() + 90;
    }
    double angle = Units.degreesToRadians(angle_degree);

    // Calculate new X position based on Trigonometry
    x += Units.inchesToMeters(inches) * Math.cos(angle);

    // Calculate new Y based on if the robot is moving right or left.
    if (inches >= 0) {
      y += Units.inchesToMeters(inches) * Math.sin(angle);
    } else {
      y -= Units.inchesToMeters(inches) * Math.sin(angle);
    }

    // Put the new X and Y on the SmartDashboard.
    SmartDashboard.putNumber("DLCA NewX", x);
    SmartDashboard.putNumber("DLCA NewY", y);

    // Return the new Pose2d.
    return new Pose2d(x, y, pose.getRotation());
  }

  @Override
  public void initialize() {
    swerve.drive(new Translation2d(0, 0), 0, true);

    // Get PID Values from SmartDashboard.
    double Py = SmartDashboard.getNumber("DLCA YLocal PID P", 3);
    double Iy = SmartDashboard.getNumber("DLCA YLocal PID I", 0.0);
    double Dy = SmartDashboard.getNumber("DLCA YLocal PID D", 0);

    // Set PID Values.
    pidy.setP(Py);
    pidy.setI(Iy);
    pidy.setD(Dy);

    // Get new Position.
    newPos = distanceToPos(target);

    // Get Y Error.
    yError = Math.abs(pidy.getError());
  }

  @Override
  public void execute() {

    // Calculate to Y Value based on weird WPIlib angle measurements.
    if (Math.abs(newPos.getRotation().getDegrees()) <= 90) {
      toY = pidy.calculate(swerve.getPose().getY(), newPos.getY());
    } else {
      toY = -pidy.calculate(swerve.getPose().getY(), newPos.getY());
    }

    SmartDashboard.putNumber("DLCA PIDyError", pidy.getError());

    SmartDashboard.putNumber("DLCA toX", toX);
    SmartDashboard.putNumber("DLCA toY", toY);

    SmartDashboard.putNumber("DLCA AngleLocal", swerve.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("DLCA XPos", swerve.getPose().getX());
    SmartDashboard.putNumber("DLCA YPos", swerve.getPose().getY());

    // Update Y Error
    yError = Math.abs(pidy.getError());


    if (yError < Units.inchesToMeters(0.5)) {
      // Deadzone
      swerve.drive(new Translation2d(0, 0), 0, true); 
    } else {
      swerve.drive(new Translation2d(0, toY * swerve.getSwerveDrive().getMaximumChassisVelocity()), 0, false);
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (yError < Units.inchesToMeters(0.5)) {
      // deadzone
      swerve.drive(new Translation2d(0, 0), 0, true);
      return true;
    } else {
      return false;
    }
  }
}