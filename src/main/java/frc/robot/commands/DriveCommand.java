// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Translation2d;
import org.jumprobotics.robot.Utilities;
import org.jumprobotics.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Joystick;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */

  public DriveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DrivetrainSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = -Robot.getRobotContainer().getPrimaryJoystick().getRawAxis(1);
    forward = Utilities.deadband(forward);
    // Square the forward stick
    forward = Math.copySign(Math.pow(forward, 2.0), forward);

    double strafe = -Robot.getRobotContainer().getPrimaryJoystick().getRawAxis(0);
    strafe = Utilities.deadband(strafe);
    // Square the strafe stick
    strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

    double rotation = -Robot.getRobotContainer().getPrimaryJoystick().getRawAxis(2);
    rotation = Utilities.deadband(rotation);
    // Square the rotation stick
    rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);
    
    DrivetrainSubsystem.getInstance().drive(new Translation2d(forward/5, strafe/5), rotation/5, true);
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
