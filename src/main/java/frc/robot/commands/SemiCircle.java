// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SemiCircle extends SequentialCommandGroup {
    PathPlannerTrajectory forward = PathPlanner.loadPath("SemiCircle", 2.0, 3.0);
            
    public SemiCircle(DrivetrainSubsystem m_drive) {/*
      addCommands(
        m_drive.dt.createCommandForTrajectory(forward, m_drive)
      );*/
      addCommands(m_drive.createCommandForTrajectory(forward, m_drive));
    }
  }