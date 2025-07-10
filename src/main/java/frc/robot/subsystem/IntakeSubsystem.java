package frc.robot.subsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX intakeMotor;
    private final DutyCycleOut intakecontrol =  new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true);
    public IntakeSubsystem(CANBus CAN) {
        intakeMotor = new TalonFX(21, CAN); 
    }
    public void setV(double voltage)
    {
        intakecontrol.Output = voltage;
        intakeMotor.setControl(intakecontrol);
    }
}
