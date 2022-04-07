package frc.robot.commands.DefenseAuto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class twotop2midD extends SequentialCommandGroup{
    PathPlannerTrajectory TtoM = PathPlanner.loadPath("2 ball top to mid defense", 8, 5);

    public twotop2midD(DrivetrainSubsystem m_drivetrainSubsystem, intake m_intakeSubsystem){
        DrivetrainSubsystem.DFLT_START_POSE = new Pose2d(6.86, 5.81, Rotation2d.fromDegrees(-58.74));
        addCommands(
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(true)),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(1)),
            // new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(-0.4)),
            new InstantCommand(() -> m_drivetrainSubsystem.setknownPose(TtoM.getInitialPose())),
            m_drivetrainSubsystem.createCommandForTrajectory(TtoM, m_drivetrainSubsystem),
            new wait(1),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(0)),
            // new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(false))
        );
    }
}
