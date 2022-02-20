// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import org.jumprobotics.robot.subsystems.Mk2SwerveDrivetrainFalcon;
import java.lang.Math;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  private static DrivetrainSubsystem instance;

  public Mk2SwerveDrivetrainFalcon drivetrain;

  public DrivetrainSubsystem() {
    double trackwidth = Constants.trackWidth;
    double wheelbase = Constants.wheelBase;
    boolean gyroscopeInverted = true;
    // module ports values are placeholders
    int[][] modulePorts = {{5, 12, 2}, // front right steer, drive, encoder
                           {6, 11, 0}, // front left steer, drive, encoder
                           {7, 4, 3}  , // back left steer, drive, encoder
                           {3, 8, 1}}; // back right steer, drive, encoder
    // zeros in offset numbers are placeholders
    double[] wheeloffsets = {-Math.toRadians(26.027723), // front right
                        -Math.toRadians(64.162519), // front left
                        -Math.toRadians(36.044750), // back left
                        -Math.toRadians(180)}; // back right

    drivetrain = new Mk2SwerveDrivetrainFalcon(trackwidth, wheelbase, wheeloffsets, modulePorts, gyroscopeInverted, false);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented){
    drivetrain.drive(translation, rotation, fieldOriented);
  }

  // Forward and strafe are bounded by [-1,1]
  public void drive(double forward, double strafe, double rotation, boolean fieldOriented){
    drivetrain.drive(new Translation2d(forward, strafe), rotation, fieldOriented);
  }

  public static DrivetrainSubsystem getInstance(){
    if(instance == null){
      instance = new DrivetrainSubsystem();
    }
      return instance;
    
  }

  public void resetGyroscope(){
    drivetrain.resetGyroscope();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    drivetrain.updateSensors();
    drivetrain.updateStates();
    drivetrain.writeToSmartDashboard();
  }
}
