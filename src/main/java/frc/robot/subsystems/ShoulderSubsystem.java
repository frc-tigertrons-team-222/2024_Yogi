// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.MathUtil;


public class ShoulderSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  public static CANSparkMax ShoulderSpark =  new CANSparkMax(32, MotorType.kBrushless);
  public static CANSparkMax ShoulderSpark2 =  new CANSparkMax(33, MotorType.kBrushless);

 private static SparkAbsoluteEncoder ShoulderEncoder = ShoulderSpark.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
public SparkPIDController ShoulderPIDController = ShoulderSpark.getPIDController();

//  private static DutyCycleEncoder ShoulderEncoder = new DutyCycleEncoder(2);
 
 public double kP = 7,
               kI = 0,
               kD = 0;
 public double setPosition=0;


  /** Creates a new DriveSubsystem. 
  * @return */
 public ShoulderSubsystem() {

  ShoulderPIDController.setP(kP);
  ShoulderPIDController.setD(kD);
  ShoulderPIDController.setI(kI);
  ShoulderPIDController.setFeedbackDevice(ShoulderEncoder);
  // ShoulderPIDController.setOutputRange(-0.7, 0.7);




  ShoulderSpark2.follow(ShoulderSpark,true);  
   
 }

 //@param set  the commanded forward movement
 
// public void ShoulderSet(double set) {
//    setPosition = set;
//    ShoulderSpeed(-MathUtil.clamp(m_pidController.calculate(ShoulderEncoder.getPosition(), setPosition),-0.5,0.5));
// }

public Command ShoulderSetPosition(double set) {
  setPosition = set;
  return run(()->ShoulderPIDController.setReference(set, CANSparkBase.ControlType.kPosition));
}

public Command ShoulderSetPositionDynamic() {;
  return run(()->ShoulderPIDController.setReference(SmartDashboard.getNumber("Shoulder Postion", 0.68), CANSparkBase.ControlType.kPosition));
}
public void ShoulderSpeed(double set) {
  ShoulderSpark.set(set);
 }

public Command ShoulderStop(){
 return runOnce(()-> ShoulderSpark.stopMotor());
} 

 public double getShoulderPosition(){
  return ShoulderEncoder.getPosition();
 }


 @Override
 public void periodic() {
   // Update the odometry in the periodic block

    SmartDashboard.putNumber("Shoulder Encoder", ShoulderEncoder.getPosition()); 
    SmartDashboard.putNumber("Shoulder Set Position",setPosition); 
    // SmartDashboard.putNumber("PID error",m_pidController.calculate(ShoulderEncoder.getPosition(), setPosition));


 }
}
