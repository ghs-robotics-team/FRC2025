package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveLocalCommandInches extends Command {
  SwerveSubsystem swerve;
  PIDController pidx;
  PIDController pidy;
  double toX;
  double toY;
  double inches;
  Pose2d newPos;
  double xError;
  double yError;

  public DriveLocalCommandInches(SwerveSubsystem swerve, double inches) {
    addRequirements(swerve);
    this.swerve = swerve;
    this.pidy = new PIDController(1, 0, 0.004); // set PID directions
    this.inches = inches;

    SmartDashboard.putNumber("DLC YLocal PID P", pidy.getP());
    SmartDashboard.putNumber("DLC YLocal PID I", pidy.getI());
    SmartDashboard.putNumber("DLC YLocal PID D", pidy.getD());
  }

  public Pose2d distanceToPos(Pose2d pose) {
    double x = pose.getX();
    double y = pose.getY();
    double angle;

    if (inches >= 0) {
      angle = Units.degreesToRadians(pose.getRotation().getDegrees() - 90);
    } else {
      angle = Units.degreesToRadians(pose.getRotation().getDegrees() + 90);
    }


    x += Units.inchesToMeters(inches) * Math.cos(angle);

    if (inches >= 0) {
      y += Units.inchesToMeters(inches) * Math.sin(angle);
    } else {
      y -= Units.inchesToMeters(inches) * Math.sin(angle);
    }

    SmartDashboard.putNumber("DLC NewX", x);
    SmartDashboard.putNumber("DLC NewY", y);

    return new Pose2d(x, y, pose.getRotation());
  }

  @Override
  public void initialize() {
    swerve.drive(new Translation2d(0, 0), 0, true);

    double Py = SmartDashboard.getNumber("DLC YLocal PID P", 3);
    double Iy = SmartDashboard.getNumber("DLC YLocal PID I", 0.0);
    double Dy = SmartDashboard.getNumber("DLC YLocal PID D", 0.004);

    pidy.setP(Py);
    pidy.setI(Iy);
    pidy.setD(Dy);

    newPos = distanceToPos(swerve.getPose());

    yError = Math.abs(pidy.getPositionError());
  }

  @Override
  public void execute() {

    if (Math.abs(newPos.getRotation().getDegrees()) <= 90) {
      toY = pidy.calculate(swerve.getPose().getY(), newPos.getY());
    } else {
      toY = -pidy.calculate(swerve.getPose().getY(), newPos.getY());
    }

    SmartDashboard.putNumber("DLC PIDy Error", pidy.getPositionError());

    SmartDashboard.putNumber("DLC toX", toX);
    SmartDashboard.putNumber("DLC toY", toY);

    SmartDashboard.putNumber("DLC AngleLocal", swerve.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("DLC XPos", swerve.getPose().getX());
    SmartDashboard.putNumber("DLC YPos", swerve.getPose().getY());

    yError = Math.abs(pidy.getPositionError());

    if (yError < Units.inchesToMeters(0.2)) {
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
    if (yError < Units.inchesToMeters(0.2)) {
      swerve.drive(new Translation2d(0, 0), 0, true);
      return true;
    } else {
      return false;
    }

  }
}