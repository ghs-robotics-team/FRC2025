// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Elevator. */
  //SparkFlex hand = new SparkFlex(16, MotorType.kBrushless); //Get ID
  TalonFX armMover = new TalonFX(18);

  double absoluteEncoder = armMover.getPosition().getValue().magnitude();
  public Arm() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void move(double amt){ 
    armMover.set(amt); // 0 to 1
    absoluteEncoder = armMover.getPosition().getValue().magnitude();
    SmartDashboard.putNumber("AA Angle", absoluteEncoder);
  }

  /*public void intake(){
    hand.set(0.5);
  }

  public void outtake() {
    hand.set(-0.5);
  }*/
  
  public double getPos(){
    return absoluteEncoder;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
