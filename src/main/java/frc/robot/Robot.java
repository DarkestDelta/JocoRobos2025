// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.util.ElasticMessages;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private ElasticMessages elasticMessages;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public Timer timer = new Timer();



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
 // Instantiate our RobotContainer.
 m_robotContainer = new RobotContainer();
 // Now that m_robotContainer is initialized, pass its ElevatorSubsystem instance to ElasticMessages.
//  elasticMessages = new ElasticMessages(m_robotContainer.getElevatorSubsystem());

//  var camera = CameraServer.startAutomaticCapture();


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

    CommandScheduler.getInstance().run();

    // elasticMessages.DisplayAllMessages();




SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
SmartDashboard.putString("Event Name", DriverStation.getEventName());
SmartDashboard.putNumber("Match Number", DriverStation.getMatchNumber());
SmartDashboard.putString("Game Message", DriverStation.getGameSpecificMessage());



  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {}

    // CommandScheduler.getInstance().disable();

    Double x = 0.250;
    Double y = 0.0;
    Double z = 0.0;
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    timer.reset();
    timer.start();
    m_autonomousCommand = m_robotContainer.m_autonomousCommand().andThen(new RunCommand(() -> m_robotContainer.m_robotDrive.drive(0, 0, 0, true), m_robotContainer.m_robotDrive));
    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }

}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
// double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
// double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees
// double ta = LimelightHelpers.getTA("");  // Target area (0% to 100% of image)
// boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?


// double drivex = 0.0;
//   if ((hasTarget) && (Math.abs(tx) > 5))
// {
// drivex = tx * .01;
// }
// CommandScheduler.getInstance().run();

    m_autonomousCommand = m_robotContainer.m_autonomousCommand().withTimeout(8);
//     System.out.println("TX: " + tx + ", Target: " + hasTarget);
    //  m_robotContainer.DriveAuto(x, y, z);
      // m_autonomousCommand = m_robotContainer.DriveAuto(x, z);
      
      // System.out.println("Command: " + m_autonomousCommand);
      // .andThen(m_robotContainer.DriveAuto(-x, y, z).withTimeout(2)).repeatedly();

    // m_robotContainer.LLalign();

  //lets goooo
  

      }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic()
 {

  }

  @Override
  public void simulationPeriodic()
  {

  }
}
