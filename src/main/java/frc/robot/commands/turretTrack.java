package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class turretTrack extends CommandBase {
    private final turret m_turret;
    private final intake m_intake;
    private String badcolor;
    private SendableChooser<String> colorpicker;
    
    /**
    * @param turret Is the subsystem used by this command.
    */
    public turretTrack(turret turret, intake intake) {
        colorpicker.addOption("Blue Alliance", "blue");
        colorpicker.addOption("Red Alliance", "red");
        m_turret = turret;
        m_intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(turret);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        badcolor = colorpicker.getSelected();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(intake.indexerState() && intake.getColorMatch().equals(badcolor)){
            m_turret.setPosition(0);
            m_turret.track(false);
        }else{
            m_turret.track(true);
        }
        
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //m_turret.reset();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

}
