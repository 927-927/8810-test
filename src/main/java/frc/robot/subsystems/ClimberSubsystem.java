package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private final TalonFX roller;
    private final TalonFX climber;
    private final double climbingV = 0.5;
    private final DutyCycleOut climbrequest = new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true);
    private final DutyCycleOut rollerrequest = new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true);
    public ClimberSubsystem(CANBus CAN)
    {
        roller = new TalonFX(22,CAN);
        climber = new TalonFX(23,CAN);
    }

    private void climbsetV(double voltage)
    {
        climbrequest.Output = voltage;
        climber.setControl(climbrequest);
    }
    
    public Command climbup()
    {
        return this.startEnd(
            () -> climbsetV(climbingV),
            () -> climbsetV(0.0)
            );
    }

    public Command climbdown()
    {
        return this.startEnd(
            () -> climbsetV(-climbingV),
            () -> climbsetV(0.0)
            );
    }
}
