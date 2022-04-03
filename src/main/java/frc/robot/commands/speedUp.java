package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.flywheel;

public class speedUp extends CommandBase {
    private final flywheel m_flywheel;
    private final double amound;

    /**
     * Creates a new ExampleCommand.
     *w
     * @param subsystem The subsystem used by this command.
     */
    public speedUp(flywheel subsystem, double a) {
        m_flywheel = subsystem;
        amound = a;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //m_flywheel.speedUp(amound);
        m_flywheel.setVelocity(amound);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_flywheel.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        
        return true;
    }
}
