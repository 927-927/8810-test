package frc.robot.subsystems;

import java.security.AlgorithmParameterGenerator;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private final TalonFX armmotor;
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    final DutyCycleOut armcontrol = new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true);
    public ArmSubsystem(CANBus canBus) {
        this.armmotor = new TalonFX(20,canBus);
        var armslot0 = new Slot0Configs();
        armslot0.kS = 0.0;
        armslot0.kV = 0.0;
        armslot0.kA = 0.0;
        armslot0.kP = 0.0;
        armslot0.kI = 0.0;
        armslot0.kD = 0.0;
        armmotor.getConfigurator().apply(armslot0);
        armmotor.setControl(armcontrol);
        armmotor.setNeutralMode(NeutralModeValue.Brake);
    }
    public void setpos(double deg)
    {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        double rot = deg*(58.33)/360;
        armmotor.setControl(m_request.withPosition(rot));
    }

    public void setV(double voltage)
    {
        armcontrol.Output = voltage;
        armmotor.setControl(armcontrol);
    }
}

