// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.lang.Math;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;





import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */

  SwerveDrivePoseEstimator m_PoseEstimator;
  public static Pose2d DFLT_START_POSE = new Pose2d();

  public static void setStartPose(Pose2d startPose){
        DFLT_START_POSE = startPose;
  }

  
  Pose2d curEstPose = new Pose2d(DFLT_START_POSE.getTranslation(), DFLT_START_POSE.getRotation());
  SwerveModuleState[] states;



  public static TrapezoidProfile.Constraints THETACONTROLLERCONSTRAINTS = 
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);



  

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  // FIXME Remove if you are using a Pigeon
  // private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // FIXME Uncomment if you are using a NavX
  private final AHRS m_navx;

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;



  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public boolean auto = false;
  public DrivetrainSubsystem() {
        m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
        
  ///CanCoders for swerve 
  //     ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration
    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
        //     tab.getLayout("Front Left Module", BuiltInLayouts.kList)
        //             .withSize(2, 4)
        //             .withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            // tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            //         .withSize(2, 4)
            //         .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            //         .withSize(2, 4)
            //         .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
        //     tab.getLayout("Back Right Module", BuiltInLayouts.kList)
        //             .withSize(2, 4)
        //             .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    var stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    var localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.1));
    var visionMeasurementStdDevs = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.1));

    m_PoseEstimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), DFLT_START_POSE,
    m_kinematics, stateStdDevs, localMeasurementStdDevs, visionMeasurementStdDevs,
    0.02);

    setknownPose(DFLT_START_POSE);
    

    states = new SwerveModuleState[] {new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()};

    
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */

  public void setknownPose(Pose2d in){
        //wants us to reset wheel encoders?
        zeroGyroscope();
        m_PoseEstimator.resetPosition(in, getGyroscopeRotation());
        curEstPose = in;
  }

  public Pose2d getpose(){
        
          return m_PoseEstimator.getEstimatedPosition();
  }

  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory, DrivetrainSubsystem m_drivetrainSubsystem){
          PPSwerveControllerCommand swerveControllerCommand =
          new PPSwerveControllerCommand(
          trajectory, 
          () -> getpose(), 
          m_kinematics, 
          new PIDController(10, 0, 0), //XPIDCONTROLLER, 
          new PIDController(10, 0, 0), //YPIDCONTROLLER, 
          new ProfiledPIDController(10, 0, 0, THETACONTROLLERCONSTRAINTS), //thetaController, 
          commandStates -> this.states = commandStates, 
          m_drivetrainSubsystem);
        return swerveControllerCommand.andThen(() -> resetStates());
  }

  public void resetStates(){
        states = new SwerveModuleState[] {new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState(),new SwerveModuleState()};
  }

  public void zeroGyroscope() {
        m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
          /*
   if (m_navx.isMagnetometerCalibrated()) {
     // We will only get valid fused headings if the magnetometer is calibrated
     return Rotation2d.fromDegrees(-m_navx.getFusedHeading());
   }*/
//
//    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
//    return Rotation2d.fromDegrees(360-m_navx.getYaw());
   //return Rotation2d.fromDegrees(-(360.0 - m_navx.getYaw()));
   return Rotation2d.fromDegrees(-m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }
/*
  public void update(){
          curEstPose = m_PoseEstimator.getEstimatedPosition();
  }*/

  public ChassisSpeeds getCS(){
          return m_chassisSpeeds;
  }


  @Override
  public void periodic() {

    //update();

    if(!auto){
        states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);  
    }else{
        m_PoseEstimator.update(getGyroscopeRotation(), states[0], states[1], states[2], states[3]);
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    
    
    
    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

   /* double FLdegree = Math.toDegrees(m_frontLeftModule.getSteerAngle());
    double FRdegree = Math.toDegrees(m_frontRightModule.getSteerAngle());
    double BLdegree = Math.toDegrees(m_backLeftModule.getSteerAngle());
    double BRdegree = Math.toDegrees(m_backRightModule.getSteerAngle());*/

    

//     SmartDashboard.putNumber("Max speed", MAX_VELOCITY_METERS_PER_SECOND);
//     SmartDashboard.putNumber("Max Rotation Speed", MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    /*SmartDashboard.putNumber("Front Left Module Angle ", FLdegree);
    SmartDashboard.putNumber("Front Right Module Angle ", FRdegree);
    SmartDashboard.putNumber("Back Left Module Angle ", BLdegree);
    SmartDashboard.putNumber("Back Right Module Angle ", BRdegree);*/
    //SmartDashboard.putNumber("estimated position x", m_PoseEstimator.getEstimatedPosition().getX());
    //SmartDashboard.putNumber("estimated position y", m_PoseEstimator.getEstimatedPosition().getY());
    //SmartDashboard.putNumber("estimated rotation", m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees());
    //SmartDashboard.putNumber("state 0", states[0].speedMetersPerSecond);

    //SmartDashboard.putString("pose", curEstPose.toString());
    //SmartDashboard.putBoolean("auto", auto);

  }
}
