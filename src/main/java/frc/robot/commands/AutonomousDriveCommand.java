// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
//import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Robot;

public class AutonomousDriveCommand extends CommandBase {
    long startTime = System.currentTimeMillis(); // in milliseconds
    long elapsedTime;
    long elapsedSeconds;
    long elapsedwholeSeconds;

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    
    double x;
    double y;
    double rotation;
    int seconds;
    
    private static int unique = 0;
    private int uid;

    public AutonomousDriveCommand(DrivetrainSubsystem drivetrainSubsystem, double x, double y, double rotation, int seconds) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.x = x;
        this.y = y;
        this.rotation = rotation;
        this.seconds = seconds;

        uid = unique;
        unique++;
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() { // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        elapsedTime = System.currentTimeMillis() - startTime;
        elapsedSeconds = elapsedTime / 1000;
        elapsedwholeSeconds = elapsedSeconds % 60;

        m_drivetrainSubsystem.drive(new ChassisSpeeds(x, y, rotation));
        
        SmartDashboard.putNumber("x: ", x);
        SmartDashboard.putNumber("y: ", y);
        SmartDashboard.putNumber("rotation: ", rotation);
        SmartDashboard.putNumber("seconds passed: ", elapsedwholeSeconds);
        SmartDashboard.putNumber("UID: ", uid);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return elapsedwholeSeconds == seconds;
    }
}