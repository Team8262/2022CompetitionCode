// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.limelight;
import frc.robot.subsystems.turret;

/** Add your docs here. */
public class CompensatedShooting {
    ChassisSpeeds cs;
    limelight limelight;
    DrivetrainSubsystem ds;
    turret turret;

    private double angle;

    private Translation2d adjPose;

    private double angleScaler;

    public CompensatedShooting(){
    }

    public CompensatedShooting SET_DRIVETRAIN(DrivetrainSubsystem ds){
        this.ds = ds;
        this.cs = ds.getCS();
        return this;
    }

    public CompensatedShooting SET_LIMELIGHT(limelight lim){
        this.limelight = lim;
        return this;
    }

    public CompensatedShooting SET_TURRET(turret t){
        this.turret = t;
        return this;
    }

    public CompensatedShooting SET_SCALERS(double angleS, double powerS){
        this.angleScaler = angleS;
        return this;
    }

    public void updateSpeeds(){
        adjPose = new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
        //This is definitely wrong, but what can you do
        angle = ds.getGyroscopeRotation().getDegrees() + this.limelight.getXOffset()+this.turret.getPosition();
        adjPose.rotateBy(Rotation2d.fromDegrees(angle));
    }




    public double getAngleAdjustment(){
        return adjPose.getX() * angleScaler;
    }


}
