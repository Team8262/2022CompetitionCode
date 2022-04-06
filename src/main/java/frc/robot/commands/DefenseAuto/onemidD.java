package frc.robot.commands.DefenseAuto;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;
import frc.robot.commands.*;


public class onemidD extends SequentialCommandGroup{
    PathPlannerTrajectory OMD = PathPlanner.loadPath("1 ball mid defense", 8, 5);
    private intake m_intakeSubsystem;

    public onemidD(DrivetrainSubsystem m_drivetrainSubsystem){
        addCommands(
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(true)),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(-1)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(-0.4)),
            new InstantCommand(() -> m_drivetrainSubsystem.setknownPose(OMD.getInitialPose())),
            m_drivetrainSubsystem.createCommandForTrajectory(OMD, m_drivetrainSubsystem),
            new wait(1),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(false))
        );
    }
}
