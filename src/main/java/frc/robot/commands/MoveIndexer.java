// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake;

public class MoveIndexer extends CommandBase {
  private intake in;
  private double val;
  /** Creates a new MoveIndexer. */
  public MoveIndexer(intake in, double val) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.in = in;
    this.val = val;
    addRequirements(in);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    in.turnStorageMotor(this.val);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    in.turnStorageMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
