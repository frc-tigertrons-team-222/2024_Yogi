// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public CANSparkFlex Beans_1=new CANSparkFlex(6, MotorType.kBrushless);
  public CANSparkFlex Beans_2=new CANSparkFlex(5, MotorType.kBrushless);
  public SparkPIDController Beans_1_PID = Beans_1.getPIDController();
  public SparkPIDController Beans_2_PID = Beans_2.getPIDController();


  public double kP = 0.00025,
                kI = 0,
                kD = 0,
                kFF = 0.00015;
  
  
  
  public Shooter() {
    // Beans_2.follow(Beans_1, true);
    Beans_1_PID.setP(kP);
    Beans_1_PID.setI(kI);
    Beans_1_PID.setD(kD);
    Beans_1_PID.setFF(kFF); //decrease 

    Beans_2_PID.setP(kP);
    Beans_2_PID.setI(kI);
    Beans_2_PID.setD(kD);
    Beans_2_PID.setFF(kFF); //decrease 

    SmartDashboard.putNumber("Shooter Speed", 2400);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command Shoot() {
    // double speed = SmartDashboard.getNumber("Shooter Speed", 0.3);
   //return runOnce(()-> Beans_1.set(-1*SmartDashboard.getNumber("Shooter Speed", 0.3))).withName("Run shooter");
    return new SequentialCommandGroup(
      runOnce(()-> Beans_1_PID.setReference(1*SmartDashboard.getNumber("Shooter Speed", 4800), CANSparkBase.ControlType.kVelocity)),
      runOnce(()-> Beans_2_PID.setReference(-1*SmartDashboard.getNumber("Shooter Speed", 4800), CANSparkBase.ControlType.kVelocity))
    );
  }

  public Command ShootDynamic(double speed ) {
    // double speed = SmartDashboard.getNumber("Shooter Speed", 0.3);
   //return runOnce(()-> Beans_1.set(-1*SmartDashboard.getNumber("Shooter Speed", 0.3))).withName("Run shooter");
    return new SequentialCommandGroup(
      runOnce(()-> Beans_1_PID.setReference(speed, CANSparkBase.ControlType.kVelocity)),
      runOnce(()-> Beans_2_PID.setReference(-1*speed, CANSparkBase.ControlType.kVelocity)));
  }

  public Command HomeShoot() {//not really home shoot far shoot
    // double speed = SmartDashboard.getNumber("Shooter Speed", 0.3);
   //return runOnce(()-> Beans_1.set(-1*SmartDashboard.getNumber("Shooter Speed", 0.3))).withName("Run shooter");
    return new SequentialCommandGroup(
      runOnce(()-> Beans_1_PID.setReference(3500, CANSparkBase.ControlType.kVelocity)),
      runOnce(()-> Beans_2_PID.setReference(1*3500, CANSparkBase.ControlType.kVelocity))
    ).repeatedly();
  }

  public Command AmpShoot() {
    // double speed = SmartDashboard.getNumber("Shooter Speed", 0.3);
   //return runOnce(()-> Beans_1.set(-1*SmartDashboard.getNumber("Shooter Speed", 0.3))).withName("Run shooter");
    return new SequentialCommandGroup(
      runOnce(()-> Beans_1_PID.setReference(500, CANSparkBase.ControlType.kVelocity)),
      runOnce(()-> Beans_2_PID.setReference(1*500, CANSparkBase.ControlType.kVelocity))
      ).repeatedly();
  }

  public Command FarShoot() {
    // double speed = SmartDashboard.getNumber("Shooter Speed", 0.3);
   //return runOnce(()-> Beans_1.set(-1*SmartDashboard.getNumber("Shooter Speed", 0.3))).withName("Run shooter");
    return new SequentialCommandGroup(
      runOnce(()-> Beans_1_PID.setReference(2400, CANSparkBase.ControlType.kVelocity)),
      runOnce(()-> Beans_2_PID.setReference(1*2400, CANSparkBase.ControlType.kVelocity))
    ).repeatedly();
  }

  public Command ShootOtherWay() {
    // double speed = SmartDashboard.getNumber("Shooter Speed", 0.3);
   return new SequentialCommandGroup(
    runOnce(()-> Beans_1_PID.setReference(-1*SmartDashboard.getNumber("Shooter Speed",  4800), CANSparkBase.ControlType.kVelocity)),
    runOnce(()-> Beans_2_PID.setReference(-1*SmartDashboard.getNumber("Shooter Speed",  4800), CANSparkBase.ControlType.kVelocity))
   );
  }

  public Command StopShoot() {
   return new SequentialCommandGroup(
    runOnce(()-> Beans_1.set(0.0)),
    runOnce(()-> Beans_2.set(0.0))
   );
  }

public Command TrapShoot() {
    return new SequentialCommandGroup(
      runOnce(()-> Beans_2_PID.setReference(-1*3200, CANSparkBase.ControlType.kVelocity))
    ).repeatedly();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Encoder Rpm", Beans_1.getEncoder().getVelocity());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
