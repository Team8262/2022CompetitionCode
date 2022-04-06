package frc.robot.commands.DefenseAuto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class twomid2topD extends SequentialCommandGroup{
    PathPlannerTrajectory MtoT = PathPlanner.loadPath("2 ball mid to top defense", 8, 5);
    private intake m_intakeSubsystem;

    public twomid2topD(DrivetrainSubsystem m_drivetrainSubsystem){
        addCommands(
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(true)),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(-1)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(-0.4)),
            new InstantCommand(() -> m_drivetrainSubsystem.setknownPose(MtoT.getInitialPose())),
            m_drivetrainSubsystem.createCommandForTrajectory(MtoT, m_drivetrainSubsystem),
            new wait(1),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(false))
        );
    }
}
