// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel;

public class flywheeltimedspin extends CommandBase {
  private flywheel m_flywheel;
  private double speed;
  private int cycles;
  private int start;
  /** Creates a new flywheeltimedspin. */
  public flywheeltimedspin(flywheel flywheel, double dist, int cycles) {
    this.m_flywheel = flywheel;
    this.speed = dist*Constants.SCALER + Constants.TRANSLATE;
    this.cycles = cycles;
    this.start = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
}

// Called when the command is initially scheduled.
@Override
public void initialize() {
    m_flywheel.setVelocity(speed);
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  this.start ++;
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  m_flywheel.setVelocity(0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
    return this.start == this.cycles;
}
}
