// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveElevator extends Command {
  /** Creates a new moveElevator. */
  Elevator elevator;
  double amt;
  public MoveElevator(Elevator elevator, double amt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.amt = amt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.move(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pos = elevator.getRelPos();
    if(amt<=0){
      if(pos <= 0){
        elevator.move(amt);
      }
      else{
        elevator.move(0);
      }
      
    }
    else{
      if(pos > -36.6){
        elevator.move(amt);
      }
      else{
        elevator.move(0);
      }
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
