// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake;

/** An example command that uses an example subsystem. */
public class forceFeedShooterTime extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final intake m_intakeSubsystem;
  private double time;
  private double start;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public forceFeedShooterTime(intake intakeSubsystem, double time) {
    this.time = time*1000;
    this.m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_intakeSubsystem.getStorageMotor().setSmartCurrentLimit(50);
    start = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.turnFeederMotor(-1);
    m_intakeSubsystem.turnStorageMotor(-0.15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.turnFeederMotor(0);
    m_intakeSubsystem.turnStorageMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - start > time;
  }
}
