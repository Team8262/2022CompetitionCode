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
import frc.robot.commands.ManualTrack;
import frc.robot.subsystems.turret;
import frc.robot.subsystems.flywheel;
import frc.robot.subsystems.intake;
import frc.robot.commands.feedShooter;
import frc.robot.commands.forceFeedShooter;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.killShooter;
import frc.robot.commands.toVarSpeed;
import frc.robot.commands.turretTrack;
import frc.robot.commands.keepFlywheelAtSpeed;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoTurn;
import frc.robot.commands.AutonomousDriveCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static Joystick primaryJoystick = new Joystick(0);
  public static Joystick turretJoystick = new Joystick(1);
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
     DoubleSupplier forwardsupp = () -> -1*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.forwardAxis)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveSpeedCap;
     

    // double strafe = getPrimaryJoystick().getRawAxis(0);
    // Square the strafe stick
    // strafe = deadband(strafe, 0.025);
    // double strafe2 = Math.copySign(Math.pow(strafe, 2.0), strafe);
    // double strafe = 0.0;
     DoubleSupplier strafesupp = () -> -1*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.strafeAxis)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * Constants.driveSpeedCap;

    // double rotation = getPrimaryJoystick().getRawAxis(2);
    // Square the rotation stick
    // rotation = deadband(rotation, 0.025);
    // double rotation2 = Math.copySign(Math.pow(rotation, 2.0), rotation);
    // double rotation = 0.0;

     DoubleSupplier rotatesupp = () -> -1*modifyAxis(getPrimaryJoystick().getRawAxis(Constants.rotationAxis)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.rotationSpeedCap;

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem, 
            forwardsupp,
            strafesupp,
            rotatesupp
            )); 
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
  private void configureButtonBindings() {
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
    track.whenInactive(new ManualTrack(turret, turretJoystick.getRawAxis(0)));
    spinFlywheel.whenHeld(new keepFlywheelAtSpeed(flywheel, aim));
    fireBall.whileHeld(new forceFeedShooter(intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SequentialCommandGroup autonCommand = new SequentialCommandGroup();

    // drive in a square, 5 seconds per side
    autonCommand.addCommands(
      new AutonomousDriveCommand(m_drivetrainSubsystem, 1.0, 0.0, 0.5, 3),
      new AutoTurn(m_drivetrainSubsystem, 95, 5),
      new AutonomousDriveCommand(m_drivetrainSubsystem, 1.0, 0.0, 0.5, 3),
      new AutoTurn(m_drivetrainSubsystem, 95, 5),
      new AutonomousDriveCommand(m_drivetrainSubsystem, 1.0, 0.0, 0.5, 3),
      new AutoTurn(m_drivetrainSubsystem, 95, 5),
      new AutonomousDriveCommand(m_drivetrainSubsystem, 1.0, 0.0, 0.5, 3),
      new AutoTurn(m_drivetrainSubsystem, 95, 5)
      //new toVarSpeed(flywheel, 100)
    );
    
    /*
    autonCommand.addCommands(new AutonomousDriveCommand(m_drivetrainSubsystem, 0.0, 1.0, 1.0, 5));
    autonCommand.addCommands(new AutonomousDriveCommand(m_drivetrainSubsystem, -1.0, 0.0, 1.0, 5));
    autonCommand.addCommands(new AutonomousDriveCommand(m_drivetrainSubsystem, 0.0, -1.0, 1.0, 5));
    // spin? for 5 seconds
    autonCommand.addCommands(new AutonomousDriveCommand(m_drivetrainSubsystem, -1.0, 1.0, 1.0, 5));
    */

    return autonCommand;
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
    value = deadband(value, 0.025);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}

