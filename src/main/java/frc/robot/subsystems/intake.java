package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;

public class intake extends SubsystemBase {

    public PneumaticHub pHub;
    Compressor pneumaticsCompressor;
    VictorSPX intakeMotor;
    CANSparkMax storageMotor;
    CANSparkMax feederMotor;
    Joystick exampleJoystick;
    boolean intakeDown = false;
    Solenoid intakeSolenoid;

    public intake() {

        pHub = new PneumaticHub(31);
        pneumaticsCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
        intakeMotor = new VictorSPX(Constants.INTAKE_MOTOR_ID);
        storageMotor = new CANSparkMax(Constants.STORAGE_MOTOR_ID, MotorType.kBrushless);
        feederMotor = new CANSparkMax(Constants.FEEDER_MOTOR_ID, MotorType.kBrushless);
        exampleJoystick = new Joystick(0);
        intakeSolenoid =  new Solenoid(31, PneumaticsModuleType.REVPH, 0);
        


    }

    @Override
    public void periodic() {
        pneumaticsCompressor.enableDigital();
        //pHub.enableCompressorDigital();
        
        intakeSolenoid.set(intakeDown);
        
        
        SmartDashboard.putBoolean("Intake Solenoid Active", intakeSolenoid.isDisabled());
    }

    @Override
    public void simulationPeriodic() {
        
    }
    
    public void setIntakeDown(boolean intakeDown) {
        this.intakeDown = intakeDown;
    }

    public CANSparkMax getStorageMotor() {
        return storageMotor;
    }

    
    public VictorSPX getIntakeMotor() {
        return intakeMotor;
    }

    public void turnFeederMotor(double speed) {
        feederMotor.set(speed);
    }

    public void turnStorageMotor(double speed) {
        storageMotor.set(speed);
    }
}
