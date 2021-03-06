// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoTurn extends CommandBase {
    long startTime; // in milliseconds
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
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() { 
        this.startTime = System.currentTimeMillis();
        this.startRotation = m_drivetrainSubsystem.getGyroscopeRotation().getDegrees();
    }

    @Override
    public void execute() { // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        elapsedTime = System.currentTimeMillis() - startTime;
        elapsedSeconds = elapsedTime / 1000;

        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, Math.copySign(3, deltaRotation)));
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