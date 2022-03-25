// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheel;
import frc.robot.subsystems.intake;

/** An example command that uses an example subsystem. */
public class feedShooter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final intake m_intakeSubsystem;
  private boolean run;
  private double start;
  private double elapsedTime = 0;
  private final flywheel m_flywheel;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public feedShooter(intake intakeSubsystem, flywheel flywheelSubsystem) {
    this.m_intakeSubsystem = intakeSubsystem;
    this.m_flywheel = flywheelSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_intakeSubsystem, this.m_flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (run){
      m_intakeSubsystem.turnFeederMotor(-1);
      m_intakeSubsystem.turnStorageMotor(-0.4);
      elapsedTime = (System.currentTimeMillis() - start)/1000;
    } else if(m_flywheel.onTarget()) {
      run = true;
      m_intakeSubsystem.turnFeederMotor(-1);
      m_intakeSubsystem.turnStorageMotor(-0.4);
      start = System.currentTimeMillis();
    }
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
    return (elapsedTime >= 1);
  }
}
