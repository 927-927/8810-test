package frc.robot.commands;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IntakeSubsystem;

public class IntakeManual extends Command {
    private IntakeSubsystem intakeSubsystem;
    private final double speed;
    public IntakeManual(IntakeSubsystem intakeSubsystem,double speed)
    {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }
    @Override
    public void initialize() {}
  
   
    @Override
    public void execute() 
    {
        intakeSubsystem.setV(speed);
    }
  
   
    @Override
    public void end(boolean interrupted) 
    {
        intakeSubsystem.setV(0);
    }
  
    @Override
    public boolean isFinished() {
        return false;
    }
}
