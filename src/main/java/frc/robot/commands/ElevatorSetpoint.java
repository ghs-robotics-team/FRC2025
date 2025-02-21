package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorSetpoint extends Command {
  /** Creates a new ArmSetpoint. */
  Elevator elevator;
  double setPoint;
  PIDController pid;

  public ElevatorSetpoint(Elevator elevator, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.setPoint = setPoint;
    this.pid = new PIDController (0.35,0,0.0005); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get PID Numbers
    double P = SmartDashboard.getNumber("Indexer-P", 0.28); 
    double I = SmartDashboard.getNumber("Indexer-I", 0.0);
    double D = SmartDashboard.getNumber("Indexer-D", 0.0005);

    // Set PID numbers
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
    
    // Get PID Controller direction for elevator to go, find current error from position.
    double direction = pid.calculate(elevator.getAbsPos(), setPoint);
    double error = pid.getPositionError();

    // If error is within 1 unit, stop moving arm.
    if (error > -1 && error < 1) {
      elevator.move(0); // deadzone
    } else {
      elevator.move(direction); // Move Arm
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
