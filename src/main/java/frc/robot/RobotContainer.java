// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.KunjamaniLifter;
import frc.robot.subsystems.limelight;
import frc.robot.commands.KunjamaniExtendLifter;
import frc.robot.subsystems.turret;
import frc.robot.subsystems.flywheel;
import frc.robot.subsystems.intake;
import frc.robot.commands.feedShooter;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.killShooter;
import frc.robot.commands.turretTrack;
import frc.robot.commands.keepFlywheelAtSpeed;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static Joystick primaryJoystick = new Joystick(0);
  public static JoystickButton lifterButton = new JoystickButton(primaryJoystick, 7);
  //public static KunjamaniLifter lifter = new KunjamaniLifter();
  private final limelight aim = new limelight();
  private final turret turret = new turret(aim);
  private final flywheel flywheel = new flywheel(aim);
  private final intake intake = new intake();

  public JoystickButton track;
  public JoystickButton IntakeButton;
  public JoystickButton spinFlywheel;
  public JoystickButton fireBall;
  public JoystickButton killShooter;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // double forward = getPrimaryJoystick().getRawAxis(1);
    // Square the forward stick
    // forward = deadband(forward, 0.025);
    // double forward2 = Math.copySign(Math.pow(forward, 2.0), forward);

    // double forward = 0.0;
     DoubleSupplier forwardsupp = () -> -2*Math.copySign(Math.pow(deadband(getPrimaryJoystick().getRawAxis(1), 0.025), 2.0), deadband(getPrimaryJoystick().getRawAxis(1), 0.025));
     

    // double strafe = getPrimaryJoystick().getRawAxis(0);
    // Square the strafe stick
    // strafe = deadband(strafe, 0.025);
    // double strafe2 = Math.copySign(Math.pow(strafe, 2.0), strafe);
    // double strafe = 0.0;
     DoubleSupplier strafesupp = () -> -2*Math.copySign(Math.pow(deadband(getPrimaryJoystick().getRawAxis(0), 0.025), 2.0), deadband(getPrimaryJoystick().getRawAxis(0), 0.025));

    // double rotation = getPrimaryJoystick().getRawAxis(2);
    // Square the rotation stick
    // rotation = deadband(rotation, 0.025);
    // double rotation2 = Math.copySign(Math.pow(rotation, 2.0), rotation);
    // double rotation = 0.0;


    
     DoubleSupplier rotatesupp = () -> -2*Math.copySign(Math.pow(deadband(getPrimaryJoystick().getRawAxis(2), 0.025), 2.0), deadband(getPrimaryJoystick().getRawAxis(2), 0.025));

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem, 
            forwardsupp,
            strafesupp,
            rotatesupp
            )); //////forwardsupp snd strafesupp in order to do field
                //////forwardsupp and strafesupp in order to do robo

    // Configure the button bindings
    //  configureButtonBindings();
  
    //DrivetrainSubsystem.getInstance().setDefaultCommand(new DriveCommand());
    configureButtonBindings();
  }

  public Joystick getPrimaryJoystick(){
    return primaryJoystick;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {/*
    new JoystickButton(primaryJoystick, Constants.zeroGyroButton).whenPressed(
       new InstantCommand(() -> DrivetrainSubsystem.getInstance().resetGyroscope())
     );*/
     //lifterButton.whileHeld(new KunjamaniExtendLifter(lifter));

     new JoystickButton(primaryJoystick, Constants.gyroButton).whenPressed(
      new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope())
    );

    IntakeButton = new JoystickButton(primaryJoystick, 7);
    track = new JoystickButton(primaryJoystick, 2);
    spinFlywheel = new JoystickButton(primaryJoystick, 2);
    fireBall = new JoystickButton(primaryJoystick, 1);
    killShooter = new JoystickButton(primaryJoystick, 10);
    
    IntakeButton.whileHeld(new IntakeControl(intake));
    killShooter.whenPressed(new killShooter(flywheel));
    track.whileHeld(new turretTrack(turret));
    spinFlywheel.whenHeld(new keepFlywheelAtSpeed(flywheel, aim));
    fireBall.whileHeld(new feedShooter(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}

