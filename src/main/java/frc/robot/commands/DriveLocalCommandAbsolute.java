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

    SmartDashboard.putNumber("DLCA YLocal PID P", pidy.getP());
    SmartDashboard.putNumber("DLCA YLocal PID I", pidy.getI());
    SmartDashboard.putNumber("DLCA YLocal PID D", pidy.getD());
  }

  public Pose2d distanceToPos(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double angle_degree;

    if (inches >= 0) {
      angle_degree = pose.getRotation().getDegrees() - 90;
    } else {
      angle_degree = pose.getRotation().getDegrees() + 90;
    }

    double angle = Units.degreesToRadians(angle_degree);

    x += Units.inchesToMeters(inches) * Math.cos(angle);

    if (inches >= 0) {
      y += Units.inchesToMeters(inches) * Math.sin(angle);
    } else {
      y -= Units.inchesToMeters(inches) * Math.sin(angle);
    }

    SmartDashboard.putNumber("DLCA NewX", x);
    SmartDashboard.putNumber("DLCA NewY", y);

    return new Pose2d(x, y, pose.getRotation());
  }

  @Override
  public void initialize() {
    swerve.drive(new Translation2d(0, 0), 0, true);

    double Py = SmartDashboard.getNumber("DLCA YLocal PID P", 3);
    double Iy = SmartDashboard.getNumber("DLCA YLocal PID I", 0.0);
    double Dy = SmartDashboard.getNumber("DLCA YLocal PID D", 0);
    pidy.setP(Py);
    pidy.setI(Iy);
    pidy.setD(Dy);

    newPos = distanceToPos(target);

    yError = Math.abs(pidy.getError());
  }

  @Override
  public void execute() {

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

    yError = Math.abs(pidy.getError());

    if (yError < Units.inchesToMeters(0.5)) {
      swerve.drive(new Translation2d(0, 0), 0, true); // Stop the robot if within tolerance
    } else {
      if (inches >= 0) { // -6
        swerve.drive(new Translation2d(0, toY * swerve.getSwerveDrive().getMaximumChassisVelocity()), 0, false);
      } else {
        swerve.drive(new Translation2d(0, toY * swerve.getSwerveDrive().getMaximumChassisVelocity()), 0, false);
      }
    }

  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (yError < Units.inchesToMeters(0.5)) {
      swerve.drive(new Translation2d(0, 0), 0, true);
      return true;
      // deadzone
    } else {
      return false;
    }
  }
}