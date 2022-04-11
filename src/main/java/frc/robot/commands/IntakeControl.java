package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake;
import com.ctre.phoenix.motorcontrol.*;


public class IntakeControl extends CommandBase {
    
    private final intake intakeSystem;

    public IntakeControl(intake intakeSystem){

        this.intakeSystem = intakeSystem;
        addRequirements(intakeSystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // add the data making
        intakeSystem.getStorageMotor().setSmartCurrentLimit(35);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSystem.setIntakeDown(true);
        intakeSystem.getIntakeMotor().set(ControlMode.PercentOutput, -1);
        intakeSystem.getStorageMotor().set(-0.3);
        intakeSystem.turnFeederMotor(1);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeSystem.setIntakeDown(false);
        intakeSystem.getIntakeMotor().set(ControlMode.PercentOutput, 0);
        intakeSystem.getStorageMotor().set(0);
        intakeSystem.turnFeederMotor(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //intakeSystem.setIntakeDown(false);
        return false;
    }
}
