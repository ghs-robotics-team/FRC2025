// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  SparkFlex Left = new SparkFlex(0, MotorType.kBrushless); //Get ID
  SparkFlex Right = new SparkFlex(1, MotorType.kBrushless);
  
  AbsoluteEncoder absoluteEncoder = Left.getAbsoluteEncoder();
  RelativeEncoder relativeEncoder = Left.getEncoder();
  double absolutepos = absoluteEncoder.getPosition(); // 0 to 1
  double totalRotations = relativeEncoder.getPosition(); // Total rotations
  public Elevator() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void move(double amt){
    Left.set(amt);
    Right.set(amt);
  }

  public double getAbsPos(){
    return absoluteEncoder.getPosition();
  }

  public double getRelPos(){
    return relativeEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
