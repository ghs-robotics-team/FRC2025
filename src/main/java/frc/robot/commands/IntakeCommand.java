// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
    private Arm arm;
    private double amt;
  
  /** Creates a new Intake_Outtake. */
  public IntakeCommand(Arm arm, double amt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.amt = amt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.intake(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.intake(amt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.outtake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
