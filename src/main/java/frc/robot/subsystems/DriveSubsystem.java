// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.DriveConstants;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;


public class DriveSubsystem extends SubsystemBase {
  // Create MaxSwerveModules
  private final MaxSwerveModule m_frontLeft = new MaxSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MaxSwerveModule m_frontRight = new MaxSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MaxSwerveModule m_rearLeft = new MaxSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MaxSwerveModule m_rearRight = new MaxSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  //
  public SwerveDriveKinematics m_driveKinematics = DriveConstants.kDriveKinematics;

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  PIDController pidControl = new PIDController(DriveConstants.kDrivingStaticAngleP,
   DriveConstants.kDrivingStaticAngleI,
   DriveConstants.kDrivingStaticAngleD);

   public HolonomicPathFollowerConfig m_holonomicConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(3.5, 0.000, 0.0), // Translation PID constants.p value was 5,but 3.5 gave consistent results
            new PIDConstants(0.8, 0, 0.0), // Rotation PID constants
            3, // Max module speed, in m/s. Was 4.5
            DriveConstants.kMaxWheelDistance, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  

  // 
  // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //     DriveConstants.kDriveKinematics,
  //     Rotation2d.fromDegrees(m_gyro.getAngle()),
  //     new SwerveModulePosition[] {
  //         m_frontLeft.getPosition(),
  //         m_frontRight.getPosition(),
  //         m_rearLeft.getPosition(),
  //         m_rearRight.getPosition()
  //     });
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(this.getAdjustedAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

      

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // configure_autobuilder(3.5,0.5);

    configure_autobuilder();
     m_frontLeft.resetEncoders();
     m_frontRight.resetEncoders();
     m_rearLeft.resetEncoders();
     m_rearRight.resetEncoders();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(this.getAdjustedAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    // CORRUPTED BY GARRETT MUAHAHAHA
    // var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
    //     fieldRelative
    //         ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle()))
    //         : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(this.getAdjustedAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command drive_command(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit){
    return new RunCommand(()->drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit));
  }

  /**
   * Method to drive the robot using using buttons.
   * Assumes gyro returns angle between 0-360
   * This method relies on the CommandSchedular to facilitate a PID loop.
   * May not work in autonomous unless used in other loop.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param angle         Angle of the robot
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void driveWithStaticAngle(double xSpeed, double ySpeed, double angle, boolean fieldRelative, boolean rateLimit){
    //Get all angles between -180->180, so pid loop knows which direction to go
    //TODO could prob make a constant for Math.PI/180 and inverse
    angle = MathUtil.angleModulus(angle*(Math.PI/180))*(180/Math.PI);
    double sensorAngle = MathUtil.angleModulus(getAdjustedAngle()*(Math.PI/180))*(180/Math.PI); //gets angle of robot, accounts for whether it is upside-down or not
    
    //make angles relative to desired angle, so setPoint becomes 0 and the sensorAngle is now relative to that. 
    sensorAngle = sensorAngle - angle;
    sensorAngle = MathUtil.angleModulus(sensorAngle*(Math.PI/180))*(180/Math.PI); //Have to do this again to make sure it's within -180->180 still

    double rotationFeedback;
    pidControl.setSetpoint(0); //technically it's to 'angle', but it's relative to 'angle' so it is always 0
    rotationFeedback=MathUtil.clamp(pidControl.calculate(sensorAngle),-1,1); 
    drive(xSpeed, ySpeed, rotationFeedback, fieldRelative, rateLimit);

     //Use PID to get the robot to turn to desired angle
     //https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/controller/PIDController.html
  }

  /**
   * Method to drive the roboto using chassis speed
   * 
   * @param givenChassisSpeeds ChassisSpeeds object with information about where you want
   *                           the robot to go. Used for pathplanner auto
   * 
   */
   public void driveUsingChassisSpeed(ChassisSpeeds givenChassisSpeeds){

    double xSpeed = givenChassisSpeeds.vxMetersPerSecond/DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeed = givenChassisSpeeds.vyMetersPerSecond/DriveConstants.kMaxSpeedMetersPerSecond;
    double rot = givenChassisSpeeds.omegaRadiansPerSecond/DriveConstants.kMaxAngularSpeed;
    drive ( xSpeed, ySpeed, rot, false, true);

   }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void Brake() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** 
   * Resets the drive encoders to currently read a position of 0.
   * 
   */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Zeroes the heading of the robot. 
   * 
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Calibrates the gyro. Used when turning on the robot.
   */
  public void calibrateGyro(){
    m_gyro.calibrate();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(this.getAdjustedAngle()).getDegrees();
  }

  /** 
   * Custom function so we can have more control over the IMU's getAngle() function, considering our roboRIO may be upside down
   * This is for this subsystem to use only, not us
   * 
   * @return Yaw axis angle in degrees (CCW positive), possibly positive or negative depending
   *         on orientation of gyro
   */
  private double getAdjustedAngle(){
    return m_gyro.getAngle(m_gyro.getYawAxis())*(DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(m_gyro.getYawAxis()) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * 
   * @return ChassisSpeeds object that tells the velocity and angle of the wheels
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {

    return m_driveKinematics.toChassisSpeeds(
      m_frontRight.getState(),
      m_frontLeft.getState(),
      m_rearRight.getState(),
      m_rearLeft.getState()

      );
  }
  
  /**
   * Method that configures the autobuilder for holonomic autonomous.
   * Uses getPose, resetOdometry, getRobotRelativeSpeeds, driveUsingChassisSpeeds
   */
  public void configure_autobuilder(){
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveUsingChassisSpeed, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        m_holonomicConfig,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }


  /**
   * Method that zeroes out the rotational velocity and translational velocity of the wheels.
   * Might be necessary to run during autonomous to make sure the wheels are commanded to spin when going into a different command
   */
  public Command stopDriveTrain(){
    //Can just call the drive() command but with 0s for x, y, and rot speed. Field Relative and Rate Limit options are arbitrary
    //In order to call the drive() command, have to use the 'this' object in order to refer to the DriveSubsystem
  return runOnce(()->this.drive(0, 0, 0, false, false)); 



  }

  


  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(this.getAdjustedAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    SmartDashboard.putNumber("Front left Angle",m_frontLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Front right Angle",m_frontRight.getPosition().angle.getDegrees() );
    SmartDashboard.putNumber("Rear left Angle", m_rearLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Rear right Angle", m_rearRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("Front left Distance",m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("Front right Distance",m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("Rear left Distance", m_rearLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("Rear right Distance", m_rearRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("Front left RPM",m_frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Front right RPM",m_frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Rear left RPM", m_rearLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Rear right RPM", m_rearRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Robot Angle", getAdjustedAngle());
    SmartDashboard.putNumber("Robot Angle Odometry", getPose().getRotation().getDegrees());

  }
}


