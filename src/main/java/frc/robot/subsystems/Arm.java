// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Globals;

public class Arm extends SubsystemBase {
  /** Creates a new Elevator. */
  SparkFlex hand = new SparkFlex(16, MotorType.kBrushless); 
  TalonFX armMotor = new TalonFX(18);

  double absoluteEncoder = armMotor.getPosition().getValue().magnitude();
  public Arm() {
    // Use addRequirements() here to declare subsystem dependencies.\
  }

  public void move(double amt){ 
    absoluteEncoder = armMotor.getRotorPosition().getValue().in(Units.Degree);// 21 Left, -21 Right for limits
    if (amt>0) {
      if (absoluteEncoder < 4500 /* 7200 at top */)   {
       armMotor.set(amt); // 0 to 1
      }
      else {
       armMotor.set(0);
      }
    }
    else if (amt<0) {
      if (absoluteEncoder > -4500 /* -7200 at top */)   {
       armMotor.set(amt); // 0 to 1
      }
      else {
       armMotor.set(0);
      }
    }
    else {
     armMotor.set(0);
    }
  }

  public void intake(double amt){
    hand.set(amt);
  }

  public void outtake(double amt) {
    hand.set(amt);
  }
  
  public double getPos(){
    return absoluteEncoder;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("AS Current Pos", armMotor.getRotorPosition().getValue().in(Units.Degree));
    SmartDashboard.putNumber("AS Target Pos", Globals.targetPos.armTarget);
  }
}
