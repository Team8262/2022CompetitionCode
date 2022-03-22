package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel;
import frc.robot.subsystems.limelight;
//import frc.robot.util.*;

public class keepFlywheelAtSpeed extends CommandBase {
    private final flywheel m_flywheel;
    private double prediction;
    //private interpolatingSheetMap list;
    private limelight camera;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public keepFlywheelAtSpeed(flywheel flywheel, limelight limelight) {
        m_flywheel = flywheel;
        camera = limelight;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        //list = new interpolatingSheetMap(Constants.DISTANCE, Constants.SPEED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //m_flywheel.setVelocity(list.getInterpolated(camera.getDistance()));
        prediction = Constants.SCALER*camera.getDistance()+Constants.TRANSLATE;
        if (prediction < 4500) {m_flywheel.setVelocity(prediction);}
        else {m_flywheel.setVelocity(4500);}
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
