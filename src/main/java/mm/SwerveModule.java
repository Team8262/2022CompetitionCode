// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package mm;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


/*
    public static final ModuleConfiguration MK4_L2 = new ModuleConfiguration(
            0.10033,
            (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
            true,
            (15.0 / 32.0) * (10.0 / 60.0),
            true
    );
*/

/** Add your docs here. */
public class SwerveModule {
    private TalonFX steerMotor;
    private TalonFX driveMotor;
    private static double nominalVoltage = 12.0;

    private static final double steerReduction = (15.0 / 32.0) * (10.0 / 60.0);

    private static final double TICKS_PER_ROTATION = 2048.0;


    final double sensorPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * steerReduction;
    final double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;


    public SwerveModule(int steerID, int driveID){
        steerMotor = new TalonFX(steerID);
        driveMotor = new TalonFX(driveID);
    }

    public double getSteerAngle(){
        return 3;
    }

    public void set(double driveVoltage, double steerAngle) {
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        double difference = steerAngle - getSteerAngle();
        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        if (difference >= Math.PI) {
            steerAngle -= 2.0 * Math.PI;
        } else if (difference < -Math.PI) {
            steerAngle += 2.0 * Math.PI;
        }
        difference = steerAngle - getSteerAngle(); // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            steerAngle += Math.PI;
            driveVoltage *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        steerAngle %= (2.0 * Math.PI);
        if (steerAngle < 0.0) {
            steerAngle += 2.0 * Math.PI;
        }

        driveMotor.set(TalonFXControlMode.PercentOutput, driveVoltage / nominalVoltage);
        steerSetReferenceAngle(steerAngle);
    }

    public void steerSetReferenceAngle(double referenceAngleRadians){
        double currentAngleRadians = steerMotor.getSelectedSensorPosition() * sensorPositionCoefficient;

        double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
        if (currentAngleRadiansMod < 0.0) {
            currentAngleRadiansMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
        double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
        if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
            adjustedReferenceAngleRadians -= 2.0 * Math.PI;
        } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
            adjustedReferenceAngleRadians += 2.0 * Math.PI;
        }

        steerMotor.set(TalonFXControlMode.Position, adjustedReferenceAngleRadians / sensorPositionCoefficient);

    }

}
