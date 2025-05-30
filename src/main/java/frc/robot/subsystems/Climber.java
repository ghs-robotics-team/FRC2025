// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  SparkMax motor = new SparkMax(19, MotorType.kBrushless); 
  double encoder = motor.getEncoder().getPosition();

  public Climber() {
  } 

  public void move(double amt){
    encoder = motor.getEncoder().getPosition();
    //amt = -amt;
    motor.set(amt);
  }

  public double getPos(){
    return encoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CS Position", motor.getEncoder().getPosition());
  }
}
