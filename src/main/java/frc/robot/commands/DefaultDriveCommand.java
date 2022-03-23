package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Robot;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private boolean robotOriented = false;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
                               DoubleSupplier  translationXSupplier,
                               DoubleSupplier  translationYSupplier,
                               DoubleSupplier  rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if(robotOriented){
            m_drivetrainSubsystem.drive(
                    new ChassisSpeeds(m_translationXSupplier.getAsDouble(),
                                        m_translationYSupplier.getAsDouble(),
                                        m_rotationSupplier.getAsDouble())
            );  
        }else{
            m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationXSupplier.getAsDouble(),
                            m_translationYSupplier.getAsDouble(),
                            m_rotationSupplier.getAsDouble(),
                            m_drivetrainSubsystem.getGyroscopeRotation()
                    )
            );  
        }
        
        SmartDashboard.putNumber("Drive Strafe Input", m_translationXSupplier.getAsDouble());
        SmartDashboard.putNumber("Drive Forward Input", m_translationYSupplier.getAsDouble());
        SmartDashboard.putNumber("Drive Rotation Input", m_rotationSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
