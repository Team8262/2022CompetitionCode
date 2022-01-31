// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Encoder;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static Command m_autonomousCommand;

  // private TalonFx talon4 = new TalonFX(4);
  // private TalonFx talon8 = new TalonFX(8);
  // private TalonFx talon11 = new TalonFX(11);
  // private TalonFx talon12 = new TalonFX(12);
  
  // private Encoder encoder4 = new Encoder(1, 2);
  // private Encoder encoder8 = new Encoder(1, 2);
  // private Encoder encoder11 = new Encoder(1, 2);
  // private Encoder encoder12 = new Encoder(1, 2);
  

  private static RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    // double posrate = 1;
    // double negrate = -1;

    // while(encoder4.getDistance()>0){
    //   talon4.set(ControlMode.PercentOutput, negrate);
    // }
    // while(encoder8.getDistance()>0){
    //   talon8.set(ControlMode.PercentOutput, negrate);
    // }
    // while(encoder11.getDistance()>0){
    //   talon11.set(ControlMode.PercentOutput, negrate);
    // }
    // while(encoder12.getDistance()>0){
    //   talon12.set(ControlMode.PercentOutput, negrate);
    // }

    // while(encoder12.getDistance()<0){
    //   talon12.set(ControlMode.PercentOutput, posrate);
    // }
    // while(encoder11.getDistance()<0){
    //   talon11.set(ControlMode.PercentOutput, posrate);
    // }
    // while(encoder8.getDistance()<0){
    //   talon8.set(ControlMode.PercentOutput, posrate);
    // }
    // while(encoder8.getDistance()<0){
    //   talon8.set(ControlMode.PercentOutput, posrate);
    // }

    // encoder4.reset();
    // encoder8.reset();
    // encoder11.reset();
    // encoder12.reset();
  }

  public static RobotContainer getRobotContainer(){
    return m_robotContainer;
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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
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
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
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
