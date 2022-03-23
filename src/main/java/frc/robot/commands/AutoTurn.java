// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
//import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;

public class AutoTurn extends CommandBase {
    long startTime = System.currentTimeMillis(); // in milliseconds
    long elapsedTime;
    long elapsedSeconds;

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    double startRotation;
    double deltaRotation;
    double seconds;

    public AutoTurn(DrivetrainSubsystem drivetrainSubsystem, double rotation, double seconds) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.deltaRotation = rotation;
        this.seconds = seconds;
        this.startRotation = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() { // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        elapsedTime = System.currentTimeMillis() - startTime;
        elapsedSeconds = elapsedTime / 1000;

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.copySign(1, deltaRotation)));
        //SmartDashboard.putNumber("rot", m_drivetrainSubsystem.getGyroscopeRotation().getDegrees()-startRotation);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return (elapsedSeconds >= seconds) || (Math.abs(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees()-startRotation) >= deltaRotation);
    }
}