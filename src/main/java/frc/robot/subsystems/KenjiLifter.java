// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
//import org.jumprobotics.robot.subsystems.Mk2SwerveDrivetrainFalcon;
import java.lang.Math;

import javax.print.attribute.standard.Compression;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Compressor;

public class KenjiLifter extends SubsystemBase {
/** Creates a new DriveSubsystem. */

private static KenjiLifter instance;
private static CANSparkMax rotationMotor1;
private static CANSparkMax rotationMotor2;
private static CANSparkMax rotationMotor3;
private static Solenoid hookSolenoid1;
private static Solenoid hookSolenoid2;
private static Solenoid liftersolenoid;
private static Compressor pneumaticsCompressor;
private static boolean hooksClosed;
private static boolean hooksState; //true means hook #1 is open, #2 is closed false means hook #2 is open, #1 is closed



    public KenjiLifter() {
        rotationMotor1 = new CANSparkMax(40, MotorType.kBrushless);
        rotationMotor2 = new CANSparkMax(41, MotorType.kBrushless);
        rotationMotor3 = new CANSparkMax(42, MotorType.kBrushless);
        liftersolenoid = new Solenoid(Constants.PH_ID, PneumaticsModuleType.REVPH, Constants.LIFTER_SOLENOID);
        hookSolenoid1 = new Solenoid(Constants.PH_ID, PneumaticsModuleType.REVPH, Constants.HOOK_SOLENOID_1);
        hookSolenoid2 = new Solenoid(Constants.PH_ID, PneumaticsModuleType.REVPH, Constants.HOOK_SOLENOID_2);
        pneumaticsCompressor = new Compressor(PneumaticsModuleType.REVPH);
        hooksClosed = false;

        rotationMotor1.setSoftLimit(SoftLimitDirection.kForward, 1/6);
        rotationMotor1.setSoftLimit(SoftLimitDirection.kReverse, 0);   
        rotationMotor2.follow(rotationMotor1, false);
        rotationMotor3.follow(rotationMotor1, false);
        pneumaticsCompressor.enableDigital();
        setLifter(false);



    }


    public static KenjiLifter getInstance(){
        if(instance == null){
            instance = new KenjiLifter();
        }
        return instance;
        
    }

    public void resetGyroscope(){

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if(hooksClosed) {
            hookSolenoid1.set(false);
            hookSolenoid2.set(false);
        } else {
            //put all other hook piston functionality here 
            //so it does not interfere when they are meant to be closed
            hookSolenoid1.set(hooksState);
            hookSolenoid2.set(!hooksState);
        }

        
    }


    public void setLifter(boolean extended) {
        liftersolenoid.set(extended);
    }

    public void stepLifter(boolean clockwise) {
        if(clockwise) {
            rotationMotor1.setSoftLimit(SoftLimitDirection.kForward, (float) (rotationMotor1.getSoftLimit(SoftLimitDirection.kForward) + 0.5));
            rotationMotor1.setSoftLimit(SoftLimitDirection.kReverse, (float) (rotationMotor1.getSoftLimit(SoftLimitDirection.kForward) + 0.5));  
        } else {
            rotationMotor1.setSoftLimit(SoftLimitDirection.kForward, (float) (rotationMotor1.getSoftLimit(SoftLimitDirection.kForward) - 0.5));
            rotationMotor1.setSoftLimit(SoftLimitDirection.kReverse, (float) (rotationMotor1.getSoftLimit(SoftLimitDirection.kForward) - 0.5));  
        }
    }

    public void turnLifter(double speed) {
        rotationMotor1.set(speed);
    }

    public void setHook1(boolean opened) {
        hookSolenoid1.set(opened);
    }

    public void setHook2(boolean opened) {
        hookSolenoid2.set(opened);
    }

    public void closeHooks(boolean close) {
        hooksClosed = close;
    }

    public void setHookState(boolean state) {
        hooksState = state;
    }

    public boolean getHookState() {
        return hooksState;
    }

    
}
