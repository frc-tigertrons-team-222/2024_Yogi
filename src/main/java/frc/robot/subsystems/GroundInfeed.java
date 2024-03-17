// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class GroundInfeed extends SubsystemBase {
  /*  */
  public CANSparkFlex InfeedMotor1=new CANSparkFlex(27, MotorType.kBrushless); //Used to load up into shooter

  public RelativeEncoder InfeedEncoder = InfeedMotor1.getEncoder();
  public SparkPIDController InfeedPIDController = InfeedMotor1.getPIDController();

  public double kP = 0.0014, 
                kI = 0.000001,
                kD = 0,
                kIz = 0,
                kFF = 0.16;  


  public GroundInfeed() { 

    InfeedPIDController.setP(kP);
    InfeedPIDController.setI(kI);
    InfeedPIDController.setD(kD);
    InfeedPIDController.setIZone(kIz);
    InfeedPIDController.setFF(kFF);
    InfeedPIDController.setOutputRange(-0.1, 0.1);
    InfeedPIDController.setFeedbackDevice(InfeedEncoder);
    InfeedPIDController.setSmartMotionAllowedClosedLoopError(0.005,0);

    InfeedMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    InfeedMotor1.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    SmartDashboard.putNumber("Ground Infeed Speed", 1);

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
  public Command SmartIntake() {
    // double speed = SmartDashboard.getNumber("Infeed Speed", 0.3);
   return runOnce(()-> InfeedMotor1.set(-1*SmartDashboard.getNumber("Ground Infeed Speed", 1))).withName("Run shooter");
  }
  
  public Command Intake(double speed) {
    // double speed = SmartDashboard.getNumber("Infeed Speed", 0.3);
   return runOnce(()-> InfeedMotor1.set(-1*speed)).withName("Run shooter");
  }

  public Command ContinuousIntake(double speed) {
    // double speed = SmartDashboard.getNumber("Infeed Speed", 0.3);
   return Intake(speed).repeatedly();
  }
  public Command ContinuousSmartIntake() {
    // double speed = SmartDashboard.getNumber("Infeed Speed", 0.3);
   return SmartIntake().repeatedly();
  }
  public Command Outtake() {
    // double speed = SmartDashboard.getNumber("Infeed Speed", 0.3);
   return runOnce(()-> InfeedMotor1.set(1*SmartDashboard.getNumber("Ground Infeed Speed", 1))).withName("Run shooter");
  }

  public Command StopInfeed() {
   return runOnce(()-> InfeedMotor1.set(0.0));
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Ground Infeed Encoder Rpm", InfeedMotor1.getEncoder().getVelocity());


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
