// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Hook extends SubsystemBase {
  /*  */
  public CANSparkMax HookMotor1=new CANSparkMax(20, MotorType.kBrushless); //Used to load up into shooter
  public CANSparkMax HookMotor2=new CANSparkMax(21, MotorType.kBrushless); //Used to load up into shooter


  public RelativeEncoder HookEncoder = HookMotor1.getEncoder();
  public SparkPIDController HookPIDController = HookMotor1.getPIDController();

  public double kP = 0.0014, 
                kI = 0.000001,
                kD = 0,
                kIz = 0,
                kFF = 0.16;  


  public Hook() {
    
    HookMotor2.follow(HookMotor1,true);

    HookPIDController.setP(kP);
    HookPIDController.setI(kI);
    HookPIDController.setD(kD);
    HookPIDController.setIZone(kIz);
    HookPIDController.setFF(kFF);
    HookPIDController.setOutputRange(-0.1, 0.1);
    HookPIDController.setFeedbackDevice(HookEncoder);
    HookPIDController.setSmartMotionAllowedClosedLoopError(0.005,0);

    SmartDashboard.putNumber("Hook Speed", 0.1);

    // SmartDashboard.putNumber("Infeed Stop kP", kP);
    // SmartDashboard.putNumber("Infeed Stop kI", kI);
    // SmartDashboard.putNumber("Infeed Stop kD", kD);
    // SmartDashboard.putNumber("Infeed Stop kFF", kFF);

   }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void SetSpeed(double speed) {
    // double speed = SmartDashboard.getNumber("Infeed Speed", 0.3);
  //  return runOnce(()-> HookMotor1.set(speed));
    HookMotor1.set(speed);
  }

  public Command StopHook() {
   return runOnce(()-> HookMotor1.set(0.0));
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Hook Encoder Rpm", HookMotor1.getEncoder().getVelocity());


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
