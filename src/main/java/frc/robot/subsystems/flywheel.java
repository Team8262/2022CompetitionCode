package frc.robot.subsystems;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class flywheel extends SubsystemBase {

    public static flywheel instance;
    private static CANSparkMax shooterMotor_1;
    //private static CANSparkMax shooterMotor_2;
    private static SparkMaxPIDController shooterPID_1;
    //private static SparkMaxPIDController shooterPID_2;
    private static double targetVelocity;
    
    
    public flywheel (limelight limelight) {
        
        shooterMotor_1 = new CANSparkMax(Constants.SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
        //shooterMotor_2 = new CANSparkMax(Constants.SHOOTER_MOTOR_2_ID, MotorType.kBrushless);

        shooterMotor_1.restoreFactoryDefaults();
        //shooterMotor_2.restoreFactoryDefaults();

        //shooterMotor_2.getEncoder().setVelocityConversionFactor(1);
        shooterMotor_1.getEncoder().setVelocityConversionFactor(1);

        //shooterMotor_1.setInverted(true);

        shooterPID_1 = shooterMotor_1.getPIDController();
        //shooterPID_2 = shooterMotor_2.getPIDController();

        shooterPID_1.setP(Constants.MASTER_SHOOTER_PID[0]);
        shooterPID_1.setI(Constants.MASTER_SHOOTER_PID[1]);
        shooterPID_1.setD(Constants.MASTER_SHOOTER_PID[2]);
        shooterPID_1.setFF(Constants.FEEDFORWARD[0]);
        /*shooterPID_2.setP(Constants.SLAVE_SHOOTER_PID[0]);
        shooterPID_2.setI(Constants.SLAVE_SHOOTER_PID[1]);
        shooterPID_2.setD(Constants.SLAVE_SHOOTER_PID[2]);
        shooterPID_2.setFF(Constants.FEEDFORWARD[1]);
        //shooterPID_2.setFeedbackDevice(sensor)

        //shooterMotor_2.follow(shooterMotor_1, false);
        //shooterPID_1.setIZone(500);
        //shooterPID_2.setIZone(500);
        */
    }

    public static flywheel getInstance(limelight aim){
        if(instance == null){
          instance = new flywheel(aim);
        }
        return instance;
    }

    public void setSpeed(double speed){
        shooterMotor_1.set(speed);
        //shooterMotor_2.set(speed);
    } 

    public void setVelocity(double velocity){
        targetVelocity = velocity;
        shooterPID_1.setReference(targetVelocity, ControlType.kVelocity);
        
        //shooterMotor_1.set(targetVelocity);     
        //shooterMotor_2.set(shooterMotor_1.getEncoder().getVelocity());
    }

    public void speedUp(double amount) {
        setVelocity(targetVelocity + amount);
    }

    public void speedDown(double amount){
        setVelocity(targetVelocity - amount);
    }

    public void stop(){
        setVelocity(0);
    }

    public double getVel1(){
        return shooterMotor_1.getEncoder().getVelocity();
    }

    public boolean onTarget(){
        return (Math.abs(shooterMotor_1.getEncoder().getVelocity()-targetVelocity) <= Constants.FLYWHEEL_TOLERANCE);
    }

    public void setMotor1(double val){
        shooterMotor_1.set(val);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel error", getVel1()-targetVelocity);
    }
}
