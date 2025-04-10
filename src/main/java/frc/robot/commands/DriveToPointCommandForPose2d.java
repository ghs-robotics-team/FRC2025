package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Globals;

public class DriveToPointCommandForPose2d extends Command {
  /** Creates a new DriveToPointCommand. */
  Pose2d point;
  Command pathfindingCommand;

  public DriveToPointCommandForPose2d(Pose2d point) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.point = point;
    this.pathfindingCommand = null;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Build Command based on Point and Constraints.
    pathfindingCommand = AutoBuilder.pathfindToPose(point, new PathConstraints(Constants.MAX_SPEED/3, Constants.MAX_SPEED/3, 2*Math.PI, 4*Math.PI), 0.0);
    
    // Setup Variables for Field and Robotpose.
    Field2d field = new Field2d();
    field.setRobotPose(point);
    SmartDashboard.putData("DTP target point", field);

    // Sends data to Globals to indicate that the robot is currently in a command.
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}