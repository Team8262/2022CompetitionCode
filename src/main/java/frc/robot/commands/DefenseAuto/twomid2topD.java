package frc.robot.commands.DefenseAuto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class twomid2topD extends SequentialCommandGroup{
    PathPlannerTrajectory MtoT = PathPlanner.loadPath("2 ball mid to top defense", 8, 5);

    public twomid2topD(DrivetrainSubsystem m_drivetrainSubsystem, intake m_intakeSubsystem){
        DrivetrainSubsystem.setStartPose(new Pose2d(5.94, 3.84, Rotation2d.fromDegrees(0)));
        addCommands(
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(true)),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(1)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(-0.4)),
            new InstantCommand(() -> m_intakeSubsystem.getIntakeMotor().set(ControlMode.PercentOutput,-1)),
            // new InstantCommand(() -> m_drivetrainSubsystem.setknownPose(MtoT.getInitialPose())),
            m_drivetrainSubsystem.createCommandForTrajectory(MtoT, m_drivetrainSubsystem),
            new wait(1),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.getIntakeMotor().set(ControlMode.PercentOutput,0)),
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(false))
        );
    }
}
