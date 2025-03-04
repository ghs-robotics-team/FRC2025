package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorSetpoint extends Command {
  /** Creates a new ArmSetpoint. */
  Elevator elevator;
  double setPoint;
  PIDController pid;
  double error;

  public ElevatorSetpoint(Elevator elevator, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    this.elevator = elevator;
    this.setPoint = setPoint;
    this.pid = new PIDController (0.10,0,0.005); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    error = pid.getPositionError();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get PID Numbers
    double P = SmartDashboard.getNumber("Indexer-P", 0.28); 
    double I = SmartDashboard.getNumber("Indexer-I", 0.0);
    double D = SmartDashboard.getNumber("Indexer-D", 0.0005);

    // Set PID numbers
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);*/
    
    // Get PID Controller direction for elevator to go, find current error from position.
    double direction = pid.calculate(elevator.getRelPos(), setPoint);
    error = pid.getPositionError();

    // If error is within 1 unit, stop moving arm.
    if (error > -0.3 && error < 0.3) {
      elevator.move(0, elevator.getRelPos()); // deadzone
    } else {
      elevator.move(-direction, elevator.getRelPos()); // Move Arm
    }
    Globals.targetPos.elevatorTarget = elevator.getRelPos();
    SmartDashboard.putNumber("ES Setpoint Error", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.move(0, elevator.getRelPos());
    Globals.targetPos.elevatorTarget = elevator.getRelPos();
    SmartDashboard.putNumber("ES Target Pos", Globals.targetPos.elevatorTarget);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return error > -0.3 && error < 0.3;
  }
}
