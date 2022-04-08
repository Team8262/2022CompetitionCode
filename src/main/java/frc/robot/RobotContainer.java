// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;
import org.opencv.core.Rect;

import frc.robot.commands.*;
import frc.robot.commands.DefenseAuto.*;
import frc.robot.subsystems.*;



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
  private final climber climber = new climber();

  private BallVision bv;


  //private final UsbCamera camera;

  public JoystickButton track;
  public JoystickButton IntakeButton;
  public JoystickButton spinFlywheel;
  public JoystickButton fireBall;
  public JoystickButton killShooter;
  public JoystickButton forceReverseIndexer;
  public JoystickButton setShoot;
  public JoystickButton manualOveride;
  public JoystickButton climb;
  public PneumaticHub ph = new PneumaticHub(31);

  public DoubleSupplier turretRot = () -> turretJoystick.getRawAxis(Constants.manualTurretAxis);

  public static final SendableChooser<Command> autoChooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //bv = new BallVision();
    //camera = CameraServer.startAutomaticCapture();
    //ph.enableCompressorDigital();
    SmartDashboard.putNumber("Shooter Offset", 0);
    ph.enableCompressorAnalog(70,120);
    SmartDashboard.putBoolean("Field Oriented", true);
    //SmartDashboard.putNumber("presure", ph.getCompressorCurrent());
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
    turret.setDefaultCommand(new turretTrack(turret, intake));

    
    autoChooser.addOption("one ball top defense", new onetopd(m_drivetrainSubsystem, intake));
    autoChooser.addOption("one ball mid defense", new onemidD(m_drivetrainSubsystem, intake));
    autoChooser.setDefaultOption("one ball bottom defense", new onebottomd(m_drivetrainSubsystem, intake));
    autoChooser.addOption("two ball mid to top defense", new twomid2topD(m_drivetrainSubsystem, intake));
    autoChooser.addOption("two ball top to mid defense", new twotop2midD(m_drivetrainSubsystem, intake));
    autoChooser.addOption("test", new testd(m_drivetrainSubsystem, intake));
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  public Joystick getPrimaryJoystick(){
    return primaryJoystick;
  }

  /*public PneumaticHub getPH(){
    return ph;
  }*/

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     //lifterButton.whileHeld(new KunjamaniExtendLifter(lifter));
    
     
     new JoystickButton(primaryJoystick, /*Constants.gyroButton*/1).whenPressed(
      new InstantCommand(() -> m_drivetrainSubsystem.zeroGyroscope())
    );

    /*
    IntakeButton = new JoystickButton(primaryJoystick, Constants.runIntake);
    climb = new JoystickButton(primaryJoystick, Constants.climbButton);

    //track = new JoystickButton(turretJoystick, Constants.trackAndSpin);
    track = new JoystickButton(primaryJoystick, 4);
    spinFlywheel = new JoystickButton(turretJoystick, Constants.trackAndSpin);
    fireBall = new JoystickButton(primaryJoystick, Constants.shootBall);
    killShooter = new JoystickButton(turretJoystick, 10);
    forceReverseIndexer = new JoystickButton(turretJoystick, Constants.forceReverseIndexer);
    //setShoot = new JoystickButton(turretJoystick, Constants.setShoot);
    setShoot = new JoystickButton(primaryJoystick, 5);
    manualOveride = new JoystickButton(turretJoystick, Constants.turretManualOverride);
    */
    IntakeButton = new JoystickButton(primaryJoystick, 6);
    climb = new JoystickButton(primaryJoystick, 2);
    spinFlywheel = new JoystickButton(primaryJoystick, 5);
    fireBall = new JoystickButton(primaryJoystick, 3);
    forceReverseIndexer = new JoystickButton(primaryJoystick,4 );


    

    IntakeButton.toggleWhenPressed(new IntakeControl(intake));
    //killShooter.whenPressed(new killShooter(flywheel));
    spinFlywheel.whenHeld(new keepFlywheelAtSpeed(flywheel, aim));
    fireBall.whileHeld(new forceFeedShooter(intake));
    //setShoot.toggleWhenPressed(/*new shootAtSpeed(flywheel, -0.7)*/ new shootAtVarSpeed(flywheel, 130));
    //manualOveride.whileHeld(new ManualTrack(turret, turretRot));
    climb.whenPressed(new InstantCommand(() -> climber.setState(!climber.getState())));


    forceReverseIndexer.whileHeld(new MoveIndexer(intake, 0.5));
    forceReverseIndexer.whileHeld(new shootAtSpeed(flywheel, 0.5));
    forceReverseIndexer.whenPressed(new InstantCommand(() -> intake.feedingBall(true)));
    forceReverseIndexer.whenReleased(new InstantCommand(() -> intake.feedingBall(false)));

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

    return autoChooser.getSelected();
    // SequentialCommandGroup autonCommand = new SequentialCommandGroup();
    // m_drivetrainSubsystem.zeroGyroscope();

    // autonCommand.addCommands( 
    //   //new AutoTurn(m_drivetrainSubsystem, 200, 1),
    //   new AutonomousDriveCommand(m_drivetrainSubsystem, -1.0, 0.0, .75, 1.9),
    //   new toVarSpeed(flywheel,120),
      // new wait(4),
    //   new feedShooter(intake, flywheel),
    //   /*new killShooter(flywheel)*/
    //   new InstantCommand(() -> flywheel.stop())
    // );

    // return autonCommand;

          /*new ParallelDeadlineGroup(
        new feedShooter(intake, flywheel), 
        new turretTrack(turret),
        new keepFlywheelAtSpeed(flywheel, aim)
      )/*,
      new ParallelRaceGroup(
        new AutonomousDriveCommand(m_drivetrainSubsystem, 1.0, 0.0, 1.2, 4),
        new IntakeControl(intake)
      ),
      new AutoTurn(m_drivetrainSubsystem, 190, 0.5),
      new ParallelDeadlineGroup(
        new feedShooter(intake, flywheel), 
        new turretTrack(turret),
        new keepFlywheelAtSpeed(flywheel, aim)
      )
      */

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

