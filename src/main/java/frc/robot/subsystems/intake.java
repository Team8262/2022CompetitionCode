package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.Mat;
import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.AnalogInput;

public class intake extends SubsystemBase {
    VictorSPX intakeMotor;
    CANSparkMax storageMotor;
    CANSparkMax feederMotor;
    Joystick exampleJoystick;
    boolean intakeDown = false;
    Solenoid intakeSolenoid;

    private static boolean feed;

    private final static I2C.Port i2cPort = I2C.Port.kOnboard;

    //private final static ColorSensorV3 colorsensor = new ColorSensorV3(i2cPort);
    private static ColorMatch colormatch;
    private final static Color blue = Constants.blueBall;
    private final static Color red = Constants.redBall;

    private final AnalogInput ultrasonic = new AnalogInput(Constants.ultrasonicPin);
    private boolean yep;


    
    public intake() {
        intakeMotor = new VictorSPX(Constants.INTAKE_MOTOR_ID);
        storageMotor = new CANSparkMax(Constants.STORAGE_MOTOR_ID, MotorType.kBrushless);
        feederMotor = new CANSparkMax(Constants.FEEDER_MOTOR_ID, MotorType.kBrushless);
        exampleJoystick = new Joystick(0);
        intakeSolenoid =  new Solenoid(31, PneumaticsModuleType.REVPH, 1);
        


        /*
        colormatch = new ColorMatch();
        colormatch.addColorMatch(red);
        colormatch.addColorMatch(blue);
        colormatch.addColorMatch(new Color(0,1,0));
        colormatch.addColorMatch(new Color(0.2,0.2,0.2));
        colormatch.addColorMatch(colorsensor.getColor());*/
    }

    @Override
    public void periodic() {
        //intakeSolenoid.set(intakeDown);
        //SmartDashboard.putString("Top Ball Color", getColorMatch());
        //boolean ballBot = getUltrasonicDist() < 40;
        //SmartDashboard.putNumber("Ultrasonic Distance (cm)", getUltrasonicDist());
        //SmartDashboard.putBoolean("Bottom Ball", ballBot);
        //SmartDashboard.putString("Raw Color", colorsensor.getColor().red + ", " + colorsensor.getColor().blue + ", " + colorsensor.getColor().green);
        //SmartDashboard.putNumber("Raw Color Dist", colorsensor.getProximity());
        /*if(exampleJoystick.getRawAxis(3) > 0.9){
            yep = true;
            feederMotor.set(0.5);
            storageMotor.set(0.1);
        }else if(yep){
            feederMotor.set(0);
            storageMotor.set(0);
            yep = false;
        }*/


    }

    /*

    public static String getColorMatch(){
        Color detectedColor = colorsensor.getColor();
        int dist = colorsensor.getProximity();
        ColorMatchResult match = colormatch.matchClosestColor(detectedColor);
        if(dist > 800){
            return "no";
        }else if(match.color == blue){
            return "blue";
        }else if(match.color == red){
            return "red";
        }
        return "no";
    }*/

    public void feedingBall(boolean state){
        feed = state;
    }

    public double getUltrasonicDist(){
        double raw_value = ultrasonic.getValue();
        double voltage_scale_factor = 5/RobotController.getVoltage5V();
        double currentDistanceCentimeters = raw_value * voltage_scale_factor * 0.125;
        return currentDistanceCentimeters; //Min Distance of 30cm
    }

    public static boolean indexerState(){
        return feed;
    }

    @Override
    public void simulationPeriodic() {
        
    }
    
    public void setIntakeDown(boolean intakeDown) {
        this.intakeDown = intakeDown;
        this.intakeSolenoid.set(intakeDown);
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
    