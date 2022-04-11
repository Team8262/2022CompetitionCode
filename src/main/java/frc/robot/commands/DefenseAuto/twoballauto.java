package frc.robot.commands.DefenseAuto;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class twoballauto extends SequentialCommandGroup { 
    // PathPlannerTrajectory OBD = PathPlanner.loadPath("1 ball bottom defense", 8, 5);
    
    public twoballauto(DrivetrainSubsystem m_drivetrainSubsystem, intake m_intakeSubsystem, flywheel flywheel, limelight limelight, turret turret){
        //DrivetrainSubsystem.setStartPose(new Pose2d(5.98, 5.15, Rotation2d.fromDegrees(92)));
        PathPlannerTrajectory OBD = PathPlanner.loadPath("Basic2Ball", 2,1/*4, 3*/);
        Pose2d temp = OBD.getInitialPose();
        temp = new Pose2d(temp.getTranslation(), temp.getRotation().rotateBy(new Rotation2d(Math.toRadians(180))));
        //m_drivetrainSubsystem.zeroGyroscope();
        //m_drivetrainSubsystem.setknownPose(OBD.getInitialPose());
        m_drivetrainSubsystem.setknownPose(temp);
        //DrivetrainSubsystem.setStartPose(OBD.getInitialPose());
        addCommands(
                    new InstantCommand(() -> turret.autoTrack = true),
                    new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(true)),
                    new InstantCommand(() -> m_intakeSubsystem.getIntakeMotor().set(ControlMode.PercentOutput, -0.3)),
                    new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(0.5)),
                    new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(-0.08)),
                    m_drivetrainSubsystem.createCommandForTrajectory(OBD, m_drivetrainSubsystem),
                    new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(false)),
                    new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(0)),
                    new InstantCommand(() -> m_intakeSubsystem.getIntakeMotor().set(ControlMode.PercentOutput, 0)),
                    new wait(0.5),
                    new InstantCommand(() -> flywheel.setVelocity(limelight.getDistance()*Constants.SCALER + Constants.TRANSLATE)),
                    new wait(2),
                    new forceFeedShooterTime(m_intakeSubsystem, 5),
                    new InstantCommand(() -> flywheel.stop()),
                    new InstantCommand(() -> turret.autoTrack = false)
                    // new feedShooter(m_intakeSubsystem, flywheel).end(true)
                    );
                    
                    
        
        /*
        addCommands(
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(true)),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(1)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(-0.4)),
            new InstantCommand(() -> m_intakeSubsystem.getIntakeMotor().set(ControlMode.PercentOutput,-1)),
            new InstantCommand(() -> m_drivetrainSubsystem.setknownPose(OBD.getInitialPose())),
            m_drivetrainSubsystem.createCommandForTrajectory(OBD, m_drivetrainSubsystem),
            new wait(1),
            new InstantCommand(() -> m_intakeSubsystem.turnFeederMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.turnStorageMotor(0)),
            new InstantCommand(() -> m_intakeSubsystem.getIntakeMotor().set(ControlMode.PercentOutput, 0)),
            new InstantCommand(() -> m_intakeSubsystem.setIntakeDown(false))
            
        );*/
    }
    
}