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

    private boolean left;
    private double spip = 0;
    public boolean autoTrack = true;

    public CANSparkMax turnMotor;
    private DutyCycleEncoder shooterEncoder;
    private DutyCycleEncoder turretEncoder;
    private SparkMaxPIDController turnMotorController;

    public double targetAngle;

    public turret(limelight limelight) {
        //turnMotor.setSmartCurrentLimit(5);
        this.camera = limelight;

        turretEncoder = new DutyCycleEncoder(1);

        //turnMotor.getEncoder().setPosition(turretEncoder.getAbsolutePosition());

        //set motors and encoders
        turnMotor = new CANSparkMax(Constants.TURN_MOTOR_ID, MotorType.kBrushless);
        turnMotor.getEncoder().setPosition(0);

        //turnMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) ((180 -(360 * shooterEncoder.get() / Constants.SHOOTER_SPROCKET_RATIO))/ (Constants.SHOOTER_SPROCKET_RATIO)));
        //turnMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) ((-180-(360 * shooterEncoder.get() / Constants.SHOOTER_SPROCKET_RATIO)) / (Constants.SHOOTER_SPROCKET_RATIO)));
        //turnMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) ((80)/ (1*Constants.SHOOTER_SPROCKET_RATIO)));
        //turnMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) ((80) / (1*Constants.SHOOTER_SPROCKET_RATIO)));
        turnMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 2.5f);
        turnMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -2.5f);


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
        //turnMotorController.setFeedbackDevice(shooterEncoder.);
        
    }

    public void track(boolean autoTrack){
        /*
        if(autoTrack) {
            turnMotorController.setReference(camera.getXOffset(), ControlType.kPosition);
        } else {
            turnMotorController.setReference(targetAngle * Constants.SHOOTER_SPROCKET_RATIO / 360, ControlType.kPosition);
        }*/
        
    }

    public void setPosition(double angle){
        turnMotorController.setReference(angle * Constants.SHOOTER_SPROCKET_RATIO / 360, ControlType.kPosition);
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

    public boolean spin(){
        left = (turnMotor.getEncoder().getPosition() > 0);
        if (this.onTarget()){
            return true;
        } else {
            turnMotorController.setReference(spip, ControlType.kPosition);
            if (left){
                spip = spip - 3 / (Constants.SHOOTER_SPROCKET_RATIO);
            } else {
                spip = spip + 3 / (Constants.SHOOTER_SPROCKET_RATIO);
            }
            return false;
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void periodic() {
        if(this.autoTrack) {
            turnMotorController.setReference(camera.getXOffset(), ControlType.kPosition);
        }
        //neoAngle = 360 * turnMotor.getEncoder().getPosition() / (Math.PI * Constants.SHOOTER_SPROCKET_RATIO);
        //revAngle = 360 * shooterEncoder.get() / Constants.SHOOTER_SPROCKET_RATIO;
        //turnMotor.getEncoder().setPosition(revAngle )* Constants.SHOOTER_SPROCKET_RATIO * Math.PI/ 360)
        //SmartDashboard.putNumber("rotations? ", revAngle);
        //SmartDashboard.putNumber("limelight offset", camera.getXOffset());
        //SmartDashboard.putNumber("Forward Soft Limit",turnMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
        //SmartDashboard.putNumber("Reverse Soft Limit", turnMotor.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));
        //SmartDashboard.putNumber("Current Turret Rotation???", turnMotor.getEncoder().getPosition());
        //SmartDashboard.putNumber("neo encoder", neoAngle);
        

    }

}