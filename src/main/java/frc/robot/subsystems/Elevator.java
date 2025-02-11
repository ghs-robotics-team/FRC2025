// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  SparkMax Left = new SparkMax(0, MotorType.kBrushless); //Get ID
  SparkMax Right = new SparkMax(1, MotorType.kBrushless);
  PIDController pid;
  
  AbsoluteEncoder absoluteEncoder = Left.getAbsoluteEncoder();
  RelativeEncoder relativeEncoder = Left.getEncoder();
  double absolutepos = absoluteEncoder.getPosition(); //0 to 1
  double totalRotations = relativeEncoder.getPosition(); // Total rotations
  public Elevator() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pid = new PIDController (0.35,0,0.0005); 
  }

  public void up(){
    Left.set(0.5);
    Right.set(0.5);
  }

  public void down(){
    Left.set(-0.5);
    Right.set(-0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double P = SmartDashboard.getNumber("Indexer-P", 0.28); 
    double I = SmartDashboard.getNumber("Indexer-I", 0.0);
    double D = SmartDashboard.getNumber("Indexer-D", 0.0005);
    double Speed =SmartDashboard.getNumber("Arm Speed", 6.5);

    // Set PID numbers
    pid.setP(P);
    pid.setI(I);
    pid.setD(D);
  }
}
