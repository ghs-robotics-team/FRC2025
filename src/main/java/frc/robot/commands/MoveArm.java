// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Globals;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArm extends Command {
  /** Creates a new ArmLeft. */
  Arm arm;
  double amt;
  public MoveArm(Arm arm, double amt) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);
    this.amt = amt;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { // 7300 left, -7300 right
    arm.move(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.move(amt);
    Globals.targetPos.armTarget = arm.getPos();
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

    return false;
  }
}
