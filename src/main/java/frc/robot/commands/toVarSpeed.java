package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheel;
import frc.robot.Constants;

public class toVarSpeed extends CommandBase {
    private final flywheel m_flywheel;
    private final double speed;
    
    /**
    * @param flywheel Is the subsystem used by this command.
    */
    public toVarSpeed(flywheel flywheel, double dist) {
        this.m_flywheel = flywheel;
        this.speed = dist*Constants.SCALER + Constants.TRANSLATE;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_flywheel.setVelocity(speed);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}