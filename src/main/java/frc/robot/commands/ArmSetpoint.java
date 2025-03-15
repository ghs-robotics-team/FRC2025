package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmSetpoint extends Command {
  /** Creates a new ArmSetpoint. */
  Arm arm;
  double setPoint;
  PIDController pid;
  double error;

  public ArmSetpoint(Arm arm, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
    this.setPoint = setPoint;
    this.pid = new PIDController (0.000065,0,0.0000025); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    error = pid.getPositionError();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get PID Controller direction for arm to go, find current error from position.
    double direction = pid.calculate(arm.getPos(), setPoint);
    error = pid.getPositionError();

    // If error is within 1 unit, stop moving arm.
    if (error > -75 && error < 75 /* goal 50 */) {
      arm.move(0); // deadzone
    } else {
      arm.move(direction); // Move Arm
    }
    Globals.targetPos.armTarget = arm.getPos();
    SmartDashboard.putNumber("AS Setpoint Error", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.move(0);
    Globals.targetPos.armTarget = arm.getPos();
    SmartDashboard.putNumber("AS Target Pos", Globals.targetPos.armTarget);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return error > -200 && error < 200;
  }
}
