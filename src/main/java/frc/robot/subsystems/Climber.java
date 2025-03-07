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
  public Climber() {//Starting: 1.08859 Lowest: 1.6973 Highest: 0.24367
  } 

  public void move(double amt){
    encoder = motor.getEncoder().getPosition();
    amt = -amt;
    if (amt>0) { // Going Down
      if (encoder < 1.6973)   { //2.1 for physical limit
       motor.set(amt); // 0 to 1
      }
      else {
        motor.set(0);
      }
    }
    else if (amt<0) {
      if (encoder > -0.8197)   { //0.37367
        motor.set(amt); // 0 to 1
      }
      else {
        motor.set(0);
      }
    }
    else {
      motor.set(0);
    }
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
