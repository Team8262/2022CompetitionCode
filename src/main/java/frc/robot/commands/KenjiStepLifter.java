// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.KenjiLifter;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class KenjiStepLifter extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final KenjiLifter lifter;
  
  
  private boolean hookState = true;
  private final boolean direction;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  // //direction = true means the robot goes up, direction = false means the robot goes down
  public KenjiStepLifter(KenjiLifter lifter, boolean direction) {
    this.lifter = lifter;
    this.direction = direction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lifter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lifter.setHookState(!lifter.getHookState());
    lifter.stepLifter(direction
    );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
