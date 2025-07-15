package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX intakeMotor;
    private final DutyCycleOut intakecontrol =  new DutyCycleOut(0.00).withOverrideBrakeDurNeutral(true);
    private BaseStatusSignal currentsupply;
    private BaseStatusSignal statorcurrentsupply;
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs()
                                            .withStatorCurrentLimit(70)
                                            .withStatorCurrentLimitEnable(true)
                                            .withSupplyCurrentLimit(50)
                                            .withSupplyCurrentLimitEnable(true);
    LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    
    public IntakeSubsystem(CANBus CAN) {
        intakeMotor = new TalonFX(21, CAN); 
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor.getConfigurator().apply(currentConfig);
        this.currentsupply = intakeMotor.getSupplyCurrent(); 
        this.statorcurrentsupply = intakeMotor.getStatorCurrent();
    }
    public void setV(double voltage)
    {
        intakecontrol.Output = voltage;
        intakeMotor.setControl(intakecontrol);
    }
    public Command intakeManual(double voltage)
    {
        return this.startEnd(
            () -> setV(voltage),
            () -> setV(-0.05)
            );
    }
    public Command intakedirect(double voltage)
    {
        return this.runOnce(
            () -> setV(voltage)
        );
    }
    public double supplycurrent()
    {
        return currentsupply.getValueAsDouble();
    }
    @Override
    public void periodic()
    {
        BaseStatusSignal.refreshAll(currentsupply,statorcurrentsupply);
    }
}
