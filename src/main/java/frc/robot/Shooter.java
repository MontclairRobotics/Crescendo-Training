// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;

public class Shooter extends SubsystemBase {
  
   //Creates the motors involved in the shooter mechanism
  public static CANSparkMax topMotor = new CANSparkMax(Ports.SHOOTER_TOP_MOTOR, MotorType.kBrushless);
  public static CANSparkMax bottomMotor = new CANSparkMax(Ports.SHOOTER_BOTTOM_MOTOR, MotorType.kBrushless);
 
  //Creates velocity PID controllers
  SparkPIDController topController = topMotor.getPIDController();
  SparkPIDController bottomController = bottomMotor.getPIDController();
 
  //Creates the encoders
  RelativeEncoder topEncoder = topMotor.getEncoder();
  RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
   
  //Create feedforward
  SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(-1,-1,-1);
  SimpleMotorFeedforward bottomFeedforwad = new SimpleMotorFeedforward(-1,-1,-1);

  /** Creates a new Shooter. */
  public Shooter() {
     topController.setP(-1);
     topController.setD(-1);
     bottomController.setP(-1);
     bottomController.setD(-1);
     topController.setReference(4000, ControlType.kVelocity);
     bottomController.setReference(4000, ControlType.kVelocity);
 


  }
  //Shooter Methods
  public static void shootAmp () {
    topMotor.set(.35);
    bottomMotor.set(.3);
  }

  public static void shootSpeaker () {
    topMotor.set(1);    
    bottomMotor.set(1);
  }

 
  
  //Shooter Commands
  public static Command speaker () {
    
    return Commands.run(() -> {shootSpeaker();});
  }

  public static Command amp () {
    return Commands.run(() -> {shootAmp();});
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }
}
