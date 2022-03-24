package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheel;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class manualSpeed extends CommandBase {
    private final flywheel m_flywheel;
    private double speed;
    private double dist;
    private final boolean dashboard;
    
    /**
    * @param flywheel Is the subsystem used by this command.
    */
    public manualSpeed(flywheel flywheel, boolean dasboard) {
        this.m_flywheel = flywheel;
        this.dashboard = dasboard;
        this.dist = 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.getNumber("Run as what distance?", dist);
        m_flywheel.setVelocity(dist*Constants.SCALER + Constants.TRANSLATE);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_flywheel.stop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}