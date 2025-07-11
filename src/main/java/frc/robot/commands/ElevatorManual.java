package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManual extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double voltage;
    public ElevatorManual(ElevatorSubsystem elevatorSubsystem,double voltage)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.voltage = voltage;
        addRequirements(elevatorSubsystem);
    }
    @Override
    public void initialize() {}
  
   
    @Override
    public void execute() 
    {
      elevatorSubsystem.setV(voltage);
    }
  
   
    @Override
    public void end(boolean interrupted) 
    {
      elevatorSubsystem.setV(0.00);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
     
}