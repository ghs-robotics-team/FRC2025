// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.controller.PIDController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmSteady extends Command {
  /** Creates a new ArmSteady. */
  Arm arm;
  PIDController pid;
  public ArmSteady(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    this.pid = new PIDController(0.0001, 0, 0);
    this.arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    // Get PID Controller direction for elevator to go, find current error from position.
    double direction = pid.calculate(arm.getPos(), Globals.targetPos.armTarget);
    double error = pid.getPositionError();

    if (error > -25 && error < 25) {
      arm.move(0); // deadzone
    } else {
      arm.move(direction); // Move Arm
    }

    // This will cause the intake roller to always run to hold the coral in.
    
    arm.intake(-0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
