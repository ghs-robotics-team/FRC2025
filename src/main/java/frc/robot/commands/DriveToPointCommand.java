package frc.robot.commands;

import java.lang.annotation.Target;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Globals;

public class DriveToPointCommand extends Command {
  /** Creates a new DriveToPointCommand. */
  TargetPoints point;
  //Command pathfindingCommand;

  public DriveToPointCommand(TargetPoints point) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.point = point;
    //this.pathfindingCommand = null;
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Command pathfindingCommand = AutoBuilder.pathfindToPose(point.get(), new PathConstraints(Constants.MAX_SPEED, 0.25 /* 3 */, 2*Math.PI, 4*Math.PI), 0.0);
    Field2d field = new Field2d();
    field.setRobotPose(point.get());
    SmartDashboard.putData("DTP target point", field);
    Globals.inPath = true;
    pathfindingCommand.andThen(new InstantCommand(() -> {
        Globals.inPath = false;
      })).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      System.out.println("Canceled");
      //pathfindingCommand.cancel();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}