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


public class onetopd  extends SequentialCommandGroup {
    PathPlannerTrajectory OTD = PathPlanner.loadPath("1 ball top defense", 8, 5);
    
    public onetopd(DrivetrainSubsystem m_drivetrainSubsystem, intake m_intakeSubsystem){
        DrivetrainSubsystem.DFLT_START_POSE = new Pose2d(6.2, 5.18, Rotation2d.fromDegrees(-22.48));
        addCommands(
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(true)),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(1)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(-0.4)),
            new InstantCommand(() -> m_intakeSubsystem.getIntakeMotor().set(ControlMode.PercentOutput,-1)),
            new InstantCommand(() -> m_drivetrainSubsystem.setknownPose(OTD.getInitialPose())),
            m_drivetrainSubsystem.createCommandForTrajectory(OTD, m_drivetrainSubsystem),
            new wait(1),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.getIntakeMotor().set(ControlMode.PercentOutput,0)),
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(false))
        );
    }
}
