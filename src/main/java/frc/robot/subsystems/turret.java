package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class turret extends SubsystemBase {
    
    limelight camera;
    public static turret instance;

    // degrees
    private double neoAngle;
    private double revAngle;

    private CANSparkMax turnMotor;
    private DutyCycleEncoder shooterEncoder;
    private SparkMaxPIDController turnMotorController;

    public turret(limelight limelight) {
        //turnMotor.setSmartCurrentLimit(5);
        this.camera = limelight;

        //set motors and encoders
        turnMotor = new CANSparkMax(Constants.TURN_MOTOR_ID, MotorType.kBrushless);
        turnMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) (180 / (Constants.SHOOTER_SPROCKET_RATIO)));
        turnMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) (-180    / (Constants.SHOOTER_SPROCKET_RATIO)));
        turnMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        turnMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        shooterEncoder = new DutyCycleEncoder(Constants.SHOOTER_ENCODER_ID);
        //shooterEncoder.setPositionOffset(Constants.SHOOTER_ENCODER_OFFSET);

        //set pid stuff
        turnMotorController = turnMotor.getPIDController();
        turnMotorController.setP(Constants.TURN_MOTOR_PID[0]);
        turnMotorController.setI(Constants.TURN_MOTOR_PID[1]);
        turnMotorController.setD(Constants.TURN_MOTOR_PID[2]);
        turnMotorController.setFF(Constants.TURN_MOTOR_PID[3]);
        turnMotor.getEncoder().setPosition(0);
        //turnMotorController.setFeedbackDevice(shooterEncoder.);
        
    }

    public void track(){
        turnMotorController.setReference(camera.getXOffset(), ControlType.kPosition);
    }

    public void stop(){
        turnMotorController.setReference(0, ControlType.kVelocity);
    }

    public void reset(){
        turnMotorController.setReference(0, ControlType.kPosition);
    }

    public boolean onTarget(){
        return camera.getXOffset() <= Constants.TURRET_TOLERANCE;
    }

    public static turret getInstance(limelight aim){
        if(instance == null){
          instance = new turret(aim);
        }
        return instance;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void periodic() {
        neoAngle = 360 * turnMotor.getEncoder().getPosition() / (Math.PI * Constants.SHOOTER_SPROCKET_RATIO);
        revAngle = 360 * shooterEncoder.get() / Constants.SHOOTER_SPROCKET_RATIO;
        SmartDashboard.putNumber("rotations? ", revAngle);
        SmartDashboard.putNumber("limelight offset", camera.getXOffset());
        SmartDashboard.putNumber("neo encoder", neoAngle);

    }

}