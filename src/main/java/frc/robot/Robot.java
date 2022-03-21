// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.subsystems.DrivetrainSubsystem;



// import com.ctre.phoenix.sensors.CANCoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static RobotContainer m_robotContainer;
  // private final DrivetrainSubsystem m_drivetrainSubsystem2 = new DrivetrainSubsystem();
  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // CANCoder frontleft = new CANCoder(3);
  // CANCoder frontright = new CANCoder(6);
  // CANCoder backright = new CANCoder(9);
  // CANCoder backleft = new CANCoder(12); 

  // // TalonFX _talon2 = new TalonFX(2);
  // TalonFX fl = new TalonFX(4);
  // // TalonFX _talon5 = new TalonFX(5);
  // TalonFX fr = new TalonFX(7);
  // // TalonFX _talon8 = new TalonFX(8);
  // TalonFX br = new TalonFX(10);
  // // TalonFX _talon11 = new TalonFX(11);
  // TalonFX bl = new TalonFX(13);

  @Override
  public void robotInit() {
    

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // m_drivetrainSubsystem2.zeroGyroscope();
    // m_drivetrainSubsystem2.resetGyroscope();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
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
  public void disabledInit() {
    // while(frontleft.getPosition()%360 != 0){
    //   fl.set(ControlMode.PercentOutput, 0.0001);
    // }
    // while(frontright.getPosition()%360 != 0){
    //   fr.set(ControlMode.PercentOutput, 0.0001);
    // }
    // while(backright.getPosition()%360 != 0){
    //   br.set(ControlMode.PercentOutput, 0.0001);
    // }
    // while(backleft.getPosition()%360 != 0){
    //   bl.set(ControlMode.PercentOutput, 0.0001);
    // }
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
  public void testPeriodic() {}
}
