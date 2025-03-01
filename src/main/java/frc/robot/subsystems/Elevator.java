// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Globals;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  SparkFlex Left = new SparkFlex(13, MotorType.kBrushless); //Get ID
  SparkFlex Right = new SparkFlex(15, MotorType.kBrushless);
  
  AbsoluteEncoder absoluteEncoder = Left.getAbsoluteEncoder();
  RelativeEncoder LrelativeEncoder = Left.getEncoder();
  RelativeEncoder RrelativeEncoder = Right.getEncoder();
  double absolutepos = absoluteEncoder.getPosition(); // 0 to 1
  double totalRotations = LrelativeEncoder.getPosition(); // Total rotations
  public Elevator() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void move(double amt, double pos){ //Bottom for both at 0, Top at -36.706 for 13, 37.828 for 15
    if(amt<=0){
      if(pos <= 0){
        Left.set(-amt);
        Right.set(amt);
      }
      else{
        Left.set(0);
        Right.set(0);
      }
    }
    else{
      if(pos > -36.6){
        Left.set(-amt);
        Right.set(amt);
      }
      else{
        Left.set(0);
        Right.set(0);
      }
    }

    SmartDashboard.putNumber("ES AbsPos", getAbsPos()); // Doesn't show up
    SmartDashboard.putNumber("ES RelPos", getRelPos());
  }

  public double getAbsPos(){
    return absoluteEncoder.getPosition();
  }

  public double getRelPos(){
    return LrelativeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("EE Target Pos", Globals.targetPos.elevatorTarget);
  }
}
