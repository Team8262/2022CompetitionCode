package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class intake extends SubsystemBase {
    VictorSPX intakeMotor;
    CANSparkMax storageMotor;
    CANSparkMax feederMotor;
    Joystick exampleJoystick;
    boolean intakeDown = false;
    Solenoid intakeSolenoid;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    private final ColorSensorV3 colorsensor = new ColorSensorV3(i2cPort);
    private final ColorMatch colormatch = new ColorMatch();


    private final Color blue = Constants.blueBall;
    private final Color red = Constants.redBall;




    public intake() {
        intakeMotor = new VictorSPX(Constants.INTAKE_MOTOR_ID);
        storageMotor = new CANSparkMax(Constants.STORAGE_MOTOR_ID, MotorType.kBrushless);
        feederMotor = new CANSparkMax(Constants.FEEDER_MOTOR_ID, MotorType.kBrushless);
        exampleJoystick = new Joystick(0);
        intakeSolenoid =  new Solenoid(31, PneumaticsModuleType.REVPH, 0);

        colormatch.addColorMatch(red);
        colormatch.addColorMatch(blue);
    }

    @Override
    public void periodic() {
        
        intakeSolenoid.set(intakeDown);
        SmartDashboard.putString("Top Ball Color", getColorMatch());
    }

    public String getColorMatch(){
        Color detectedColor = colorsensor.getColor();
        ColorMatchResult match = colormatch.matchClosestColor(detectedColor);
        if(match.color == blue){
            return "blue";
        }else if(match.color == red){
            return "red";
        }
        return "no";
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
