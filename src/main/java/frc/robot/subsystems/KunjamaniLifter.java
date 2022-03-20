// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import org.jumprobotics.robot.subsystems.Mk2SwerveDrivetrainFalcon;
import java.lang.Math;

import javax.print.attribute.standard.Compression;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Compressor;

public class KunjamaniLifter extends SubsystemBase {
/** Creates a new DriveSubsystem. */

private static KunjamaniLifter instance;
private static Solenoid hookSolenoid1;
private static Solenoid hookSolenoid2;
private static Solenoid liftersolenoid;
private static Compressor pneumaticsCompressor;



    public KunjamaniLifter() {
        liftersolenoid = new Solenoid(Constants.PH_ID, PneumaticsModuleType.REVPH, Constants.LIFTER_SOLENOID);
        pneumaticsCompressor = new Compressor(PneumaticsModuleType.REVPH);

        pneumaticsCompressor.enableDigital();
        setLifter(false);



    }


    public static KunjamaniLifter getInstance(){
        if(instance == null){
            instance = new KunjamaniLifter();
        }
        return instance;
        
    }

    public void resetGyroscope(){

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }


    public void setLifter(boolean extended) {
        liftersolenoid.set(extended);
    }
}
