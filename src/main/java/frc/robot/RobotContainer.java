// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.List;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WristInfeed;
import frc.robot.subsystems.GroundInfeed;
import frc.robot.subsystems.Infeed;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Solenoid;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hook;
import frc.robot.subsystems.Blinkin;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.LimelightConstants;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShoulderSubsystem m_ShoulderSubsystem = new ShoulderSubsystem();
  private final Infeed m_Infeed = new Infeed();
  private final Shooter m_Shooter = new Shooter();
  private final Hook m_Hook = new Hook();
  private final Autos m_autos = new Autos(m_robotDrive,m_Infeed,m_ShoulderSubsystem, m_Shooter);
  private final LimelightSubsystem m_Limelight = new LimelightSubsystem();
  private final Solenoid m_Solenoid=new Solenoid(); 
  // The driver's controller
//   XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandXboxController m_driverController2 = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
          

          new ParallelCommandGroup(
            new RunCommand(
              () -> m_robotDrive.drive(
                  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                  true, true),
              m_robotDrive), 
              climbChain()
            
          )
        );

    // Register auto commands
    loadAutoCommands();

    
    SmartDashboard.putNumber("Shoulder Position", 0.76);


    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj2.command.button.CommandXboxController}, and then calling
   * one of the {@link edu.wpi.first.wpilibj2.command.button.Trigger} methods.
   */
  private void configureButtonBindings() {
    // m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_robotDrive.setX(),
    //                                                                 m_robotDrive));

    ///These buttons have the robot drive at a specific angle///
    m_driverController.y().whileTrue(new RunCommand(()->m_robotDrive.driveWithStaticAngle(
                                                                                        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                                                                        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                                                                        0,
                                                                                        true,
                                                                                        true),
                                                                                        m_robotDrive));
    m_driverController.x().whileTrue(new RunCommand(()->m_robotDrive.driveWithStaticAngle(
                                                                                        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                                                                        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                                                                        90,
                                                                                        true,
                                                                                        true),
                                                                                        m_robotDrive));
    m_driverController.a().whileTrue(new RunCommand(()->m_robotDrive.driveWithStaticAngle(
                                                                                        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                                                                        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                                                                        180,
                                                                                        true,
                                                                                        true),
                                                                                        m_robotDrive));
    m_driverController.b().whileTrue(new RunCommand(()->m_robotDrive.driveWithStaticAngle(
                                                                                        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                                                                        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                                                                        270,
                                                                                        true,
                                                                                        true),
                                                                                        m_robotDrive));

    

    m_driverController.rightTrigger().whileTrue(m_Infeed.GoOutfeed()).whileFalse(m_Infeed.StopInfeed());
    // m_driverController.leftTrigger().onTrue(m_Infeed.Intake().alongWith(Commands.run(()->m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.home)).repeatedly())).whileFalse(m_Infeed.StopInfeed().andThen(m_ShoulderSubsystem.ShoulderStop()));
    m_driverController.leftTrigger().onTrue(this.FullIntake(1))
                                    .whileFalse(m_Infeed.StopInfeed()
                                    .andThen(m_ShoulderSubsystem.ShoulderStop()));

    m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_robotDrive.Brake(),
                                                                    m_robotDrive));
    m_driverController.leftBumper().whileTrue(m_Infeed.Outtake()).whileFalse(m_Infeed.StopInfeed());
    m_driverController.start().whileTrue(new RunCommand(()->m_robotDrive.zeroHeading()));
    m_driverController.back().whileTrue(m_Infeed.WristOuttake()).whileFalse(m_Infeed.StopInfeed());
   // m_driverController.rightStick().whileTrue(new ParallelCommandGroup(turnTowardsTag(),aimAtTag()));
    m_driverController.rightStick().whileTrue(turnTowardsTag().alongWith(aimAtTag()));


    //NOTE was onTrue
    //m_driverController2.b().whileTrue(m_ShoulderSubsystem.ShoulderSet(ShoulderConstants.forward));
    m_driverController2.a().onTrue(m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.realfarshot).alongWith(m_Shooter.HomeShoot())).whileFalse(m_ShoulderSubsystem.ShoulderStop().andThen(m_Shooter.StopShoot()));
    m_driverController2.b().onTrue(m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.home)).whileFalse(m_ShoulderSubsystem.ShoulderStop().andThen(m_Shooter.StopShoot()));
    m_driverController2.y().onTrue(m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.farshot).alongWith(m_Shooter.FarShoot())).whileFalse(m_ShoulderSubsystem.ShoulderStop().andThen(m_Shooter.StopShoot()));
    m_driverController2.x().onTrue(m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.amp).alongWith(m_Shooter.AmpShoot())).whileFalse(m_ShoulderSubsystem.ShoulderStop().andThen(m_Shooter.StopShoot()));
    m_driverController2.rightTrigger().whileTrue(m_Shooter.Shoot()).whileFalse(m_Shooter.StopShoot());
    m_driverController2.leftTrigger().whileTrue(m_Shooter.ShootOtherWay()).whileFalse(m_Shooter.StopShoot());
    m_driverController2.rightBumper().onTrue(m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.amp).alongWith(m_Shooter.TrapShoot())).whileFalse(m_Shooter.StopShoot());
    m_driverController2.leftBumper().onTrue(m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.amp)).whileFalse(m_Shooter.StopShoot());
    m_driverController2.start().whileTrue(m_Solenoid.switchOn()).whileFalse(m_Solenoid.switchOff()); //Should make it on true and then consequently shut it off so it can't be a sustained
// 


    

   
    // m_driverController2.rightTrigger().whileTrue(m_ShoulderSubsystem.ShoulderSet(0.1));
    // m_driverController2.leftTrigger().whileTrue(m_ShoulderSubsystem.ShoulderSet(-0.1));

    // m_driverController2.rightStick().whileTrue(m_ShoulderSubsystem.ShoulderSpeed(MathUtil.clamp(MathUtil.applyDeadband(m_driverController2.getRightX(), OIConstants.kDriveDeadband),-0.3,0.3)));
    
      
      ////////////////////////////////////////////////////////////
  }


  /*
   * Use this method to reset the reference frame of the gyro. 
   * This will make it so that the current forward facing direction 
   * of the gyro is 0.
   */
  public void calibrateGyro(){
    m_robotDrive.calibrateGyro();
  }

  public void resetHeading(){
    m_robotDrive.zeroHeading();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    //An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, .5)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1, 0.51, new Rotation2d(0)),
        config);


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  public Command getPathPlannerAuto(String filename){
    return m_autos.getPathPlannerAuto(filename);
  }

  public Command makePathPlannerAuto(String filename){
    return m_autos.makePathPlannerAuto(filename);
  }


  public DriveSubsystem getDriverSubsystem(){
    return m_robotDrive;
  }

  public Autos getAutos(){
    return m_autos;
  }

  public Command initializeTeleop(){
    return new SequentialCommandGroup(
      m_robotDrive.stopDriveTrain(),
      m_Shooter.StopShoot(),
      m_Infeed.StopInfeed(),
      m_Solenoid.switchOff()
    );
  }

  public Command FullIntake(double speed){
    return m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.home)
            .until(()->m_ShoulderSubsystem.getShoulderPosition()<0.718)
            .andThen(m_Infeed.Intake(speed));
  }
  
  public void loadAutoCommands(){
    NamedCommands.registerCommand("Shooter Shoot", m_Shooter.Shoot().repeatedly());
    NamedCommands.registerCommand("Shooter Shoot In", m_Shooter.ShootOtherWay().repeatedly());
    NamedCommands.registerCommand("Infeed Intake", this.FullIntake(0.6)); //not repeated, because it is natively repeated
    NamedCommands.registerCommand("Infeed Outtake",m_Infeed.Outtake().repeatedly());
    NamedCommands.registerCommand("Drivetrain Stop!",m_robotDrive.stopDriveTrain());
    NamedCommands.registerCommand("Infeed Shoot",m_Infeed.GoOutfeed().repeatedly());
    NamedCommands.registerCommand("Shoulder Amp Pose",m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.amp).alongWith(m_Shooter.AmpShoot()));
    NamedCommands.registerCommand("Shoulder Home Pose",m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.home).alongWith(m_Shooter.HomeShoot()));
    NamedCommands.registerCommand("Shoulder Farshot Pose",m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.realfarshot).alongWith(m_Shooter.HomeShoot()));
    NamedCommands.registerCommand("Shoulder 0.66",m_ShoulderSubsystem.ShoulderSetPosition(0.66).alongWith(m_Shooter.HomeShoot()));
    NamedCommands.registerCommand("Shoulder 0.68",m_ShoulderSubsystem.ShoulderSetPosition(0.68).alongWith(m_Shooter.HomeShoot()));
    NamedCommands.registerCommand("Shoulder 0.72",m_ShoulderSubsystem.ShoulderSetPosition(0.72).alongWith(m_Shooter.HomeShoot()));
    NamedCommands.registerCommand("Shoulder 0.88",m_ShoulderSubsystem.ShoulderSetPosition(0.88).alongWith(m_Shooter.HomeShoot()));
    NamedCommands.registerCommand("Continuous Intake",m_Infeed.IntakeContinuously().alongWith(m_Shooter.Shoot().repeatedly()));

      for (double i = 0.66; i<=0.88; i=i+0.001){
        NamedCommands.registerCommand("Shoulder "+String.format("%.3f",i),m_ShoulderSubsystem.ShoulderSetPosition(i).alongWith(m_Shooter.HomeShoot()));
        //Commands.print("Shoulder "+String.format("%.3f",i)).schedule();
      // System.out.println("Shoulder "+i);
    }
    for (double i = 0.000; i<=(ShoulderConstants.amp-ShoulderConstants.home); i=i+0.001){
        NamedCommands.registerCommand("Shoulder Home+"+String.format("%.3f",i),m_ShoulderSubsystem.ShoulderSetPosition(ShoulderConstants.home+i).alongWith(m_Shooter.HomeShoot())); //3500 RPM
        //Commands.print("Shoulder "+String.format("%.3f",i)).schedule();
      // System.out.println("Shoulder "+i);
    }
    for (int j = 1000; j<= 5000; j=j+200){
        NamedCommands.registerCommand("Shooter Speed "+String.format("%d",j),m_Shooter.ShootDynamic(j));
        //Commands.print("Shoulder "+String.format("%.3f",i)).schedule();
      // System.out.println("Shoulder "+i);
    }

     for (double i = 0.66; i<=0.88; i=i+0.001){
        NamedCommands.registerCommand("Shoulder "+String.format("%.3f",i)+" No Shoot",m_ShoulderSubsystem.ShoulderSetPosition(i));
        //Commands.print("Shoulder "+String.format("%.3f",i)).schedule();
      // System.out.println("Shoulder "+i);
    }
    
  }

  public Command turnTowardsTag(){
    return new RunCommand(()->m_robotDrive.drive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                            -MathUtil.clamp(m_Limelight.getXPosition()*limelightadjustconstants.kP+limelightadjustconstants.kFF,-0.1,0.1),true,false)); 
  }

  public Command aimAtTag(){
    // return m_robotDrive.drive_command(0,0,MathUtil.clamp(m_Limelight.getXPosition()/limelightadjustconstants.kP,-0.5,0.5),true,false); 
    return m_ShoulderSubsystem.ShoulderSetPosition(MathUtil.clamp(ShoulderConstants.home+(LimelightConstants.resting_angle-(LimelightConstants.angular_offset+m_Limelight.getYPosition()))/360, ShoulderConstants.home, ShoulderConstants.amp)); 
  }
  public boolean isTracking(){
    return m_driverController.rightStick().getAsBoolean();
  }

public Command climbChain(){
  return new ParallelCommandGroup(new RunCommand(
              ()->m_Hook.SetSpeed(-MathUtil.clamp(m_driverController2.getRightY(),-0.8,0.8)),
              m_Hook),
              runOnce(()->m_ShoulderSubsystem.ShoulderSpeed(-MathUtil.clamp(m_driverController2.getRightY(),-0.8,0.8)))
              );
}

  public Infeed getInfeed(){
      return m_Infeed;
    }
public LimelightSubsystem getLimelight(){
    return m_Limelight;
    }
  public final class AutoCommands{
    
  public Command Shooter_Shoot(){
      return m_Shooter.Shoot().repeatedly();
    }

  public Command Infeed_Intake(){
      return m_Infeed.Intake(0.8).repeatedly();
    }
  }

  public final class limelightadjustconstants{
    static double  kP=0.01,
    kI=0,
    kD=0,
    kFF=0;
  }
  
}
