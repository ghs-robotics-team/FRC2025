package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public enum TargetPoints {
    TOP_LEFT(new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120))),
    TOP_RIGHT(new Pose2d(Units.inchesToMeters(193.1), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60))),
    RIGHT(new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(0))), 
    LEFT(new Pose2d(Units.inchesToMeters(144), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(180))),
    BOTTOM_RIGHT(new Pose2d(Units.inchesToMeters(193.1), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300))),
    BOTTOM_LEFT(new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240))),
    TOP_STATION(new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.2), Rotation2d.fromDegrees(306))),
    BOTTOM_STATION(new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.8), Rotation2d.fromDegrees(54))),
    TEST_LEFT_STATION(new Pose2d(Units.inchesToMeters(144), Units.inchesToMeters(158.5-(12.94/2)), Rotation2d.fromDegrees(180)));

    public Pose2d pose;
    private TargetPoints(Pose2d pose){
        this.pose = pose;
    }
    public Pose2d distanceFromTag(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        x += Units.inchesToMeters(18)*Math.cos(pose.getRotation().getRadians()); //24
        y += Units.inchesToMeters(18)*Math.sin(pose.getRotation().getRadians()); //24
        return new Pose2d(x,y,Rotation2d.fromDegrees(180).plus(pose.getRotation()));
    }
    public Pose2d get(){
        Pose2d newpose = distanceFromTag(pose);
        if(DriverStation.getAlliance().get() == Alliance.Red){
            return new Pose2d(newpose.getX()+ 8.57,newpose.getY(), newpose.getRotation());
        }else{
            return newpose;
        }
    }
}