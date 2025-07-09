package frc.robot.commands;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.ElevatorSubsystem;
public class ElevatorPIDtest extends Command{
    private final ElevatorSubsystem elevatorSubsystem;
    private final double pos;
    public ElevatorPIDtest(ElevatorSubsystem elevatorSubsystem,double pos)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        this.pos = pos;
        addRequirements(elevatorSubsystem);

    }
    @Override
    public void initialize() {elevatorSubsystem.movetoheight(pos);}
  
   
    @Override
    public void execute() 
    {
        SignalLogger.writeDouble("height", elevatorSubsystem.getheight());
    }
  
   
    @Override
    public void end(boolean interrupted) 
    {
        elevatorSubsystem.setV(0);
    }
  
    @Override
    public boolean isFinished() {
        return elevatorSubsystem.reached(pos);
    }
}
