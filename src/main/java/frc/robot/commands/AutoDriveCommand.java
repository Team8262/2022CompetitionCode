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

public class AutoDriveCommand extends CommandBase {
  /** Creates a new AutoDriveCommand. */
  long startTime = System.currentTimeMillis(); // in milliseconds
  long elapsedTime;
  long elapsedSeconds;
  long wholeSeconds;

  Translation2d translation;
  double rotation;
  int seconds;
  //int counter;
  private static int unique = 0;
  private int uid;
  public AutoDriveCommand(double x, double y, double rotate, int seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DrivetrainSubsystem.getInstance());
    this.translation = new Translation2d(x, y);
    this.rotation = rotate;
    this.seconds = seconds;
    //this.counter = 0;

    uid = unique;
    unique++;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DrivetrainSubsystem.getInstance().drive(translation, rotation, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elapsedTime = System.currentTimeMillis() - startTime;
    elapsedSeconds = elapsedTime / 1000;
    wholeSeconds = elapsedSeconds % 60;
    //counter ++;
    System.out.println("UID: " + uid);
    //System.out.println("Counter: " + counter);
    System.out.println("Seconds passed: " + wholeSeconds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DrivetrainSubsystem.getInstance().drive(new Translation2d(0,0), 0, true);
    //counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wholeSeconds == seconds;
  }
}