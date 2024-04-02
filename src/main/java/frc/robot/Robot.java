// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Autos;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.LimelightSubsystem;





/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Autos m_autos;

  private Blinkin m_Blinkin;

  SendableChooser<Command> m_chooser = new SendableChooser();
  private LimelightSubsystem m_Limelight;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_autos = m_robotContainer.getAutos();
    m_Blinkin = new Blinkin();
    m_Limelight=m_robotContainer.getLimelight();
    m_robotContainer.calibrateGyro();
    //CameraServer.startAutomaticCapture();
    //***Get the autos and add to shuffleboard***//
    //returns names of pathplanner autos via Strings
    // List<String> auto_command_files = m_autos.getAutoNames(); 

    //Make the first .auto file the default option in shuffleboard
    //m_chooser.setDefaultOption("Amp Shot, Long Shot 1st Note", m_autos.makePathPlannerAuto("Amp Shot, Long Shot 1st Note"));
    m_chooser.setDefaultOption("3 center note-Erik", m_autos.makePathPlannerAuto("3 center note-Erik"));
    m_chooser.addOption("Amp Shot, Long Shot 2nd Note", m_autos.makePathPlannerAuto("Amp Shot, Long Shot 2nd Note"));
    m_chooser.addOption("SPSLRS", m_autos.makePathPlannerAuto("SPSLRS"));
    m_chooser.addOption("Test Amp Shot, Long Shot 1st Note", m_autos.makePathPlannerAuto("Test Amp Shot, Long Shot 1st Note"));
    m_chooser.addOption("Amp Shot, Long Shot 1st Note REL", m_autos.makePathPlannerAuto("Amp Shot, Long Shot 1st Note REL"));
    //m_chooser.addOption("3 center note-Erik REL", m_autos.makePathPlannerAuto("3 center note-Erik REL"));
    m_chooser.addOption("Amp long shot 1st note def", m_autos.makePathPlannerAuto("Amp long shot 1st note def"));
    m_chooser.addOption("Defensive Middle", m_autos.makePathPlannerAuto("Defensive Middle"));

    // m_chooser.addOption("Defensive Middle ", m_autos.makePathPlannerAuto("Defensive Middle "));


    //Add each .auto file as options
    // int i;
    // for(i=1;i<auto_command_files.size();i++){
    //   m_chooser.addOption(auto_command_files.get(i), m_autos.makePathPlannerAuto(auto_command_files.get(i)));
    // }

    //Add the auto options to the tab "Autonomous" in shuffleboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);

  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.makePathPlannerAuto("pathplanner/generatedJSON/Test_Path_3.json");
    // m_autonomousCommand = m_robotContainer.makePathPlannerAuto("test_auto_1");
        // m_autonomousCommand = m_robotContainer.makePathPlannerAuto("spin_in_place");

    m_autonomousCommand = m_chooser.getSelected();

    m_robotContainer.initializeTeleop().schedule();
    
    m_robotContainer.resetHeading();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.calibrateGyro();
    m_robotContainer.resetHeading();

    m_robotContainer.initializeTeleop().schedule();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(m_robotContainer.getInfeed().getResult()){
      m_Blinkin.noNoteLoaded();
    }
 else if ((!m_robotContainer.getInfeed().getResult())&(Math.abs(m_Limelight.getXPosition())<3)&(m_robotContainer.isTracking())){
      m_Blinkin.limelightSighted();
    }

    else if (!m_robotContainer.getInfeed().getResult()) {
      m_Blinkin.noteLoaded();
    } 
   
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
