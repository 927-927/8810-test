package frc.robot.subsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX elemotor1;
    private final TalonFX elemotor2;
    private final double dia = 38.2*Math.PI;
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    final DutyCycleOut elecontrol = new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true);
    final PositionVoltage request = new PositionVoltage(0).withSlot(0).withOverrideBrakeDurNeutral(true);
    private final BaseStatusSignal positionSignal;
    
    public ElevatorSubsystem(CANBus CAN,double curlimit)
    {
        this.elemotor1 = new TalonFX(8,CAN);
        this.elemotor2 = new TalonFX(9,CAN);
        currentConfig.StatorCurrentLimitEnable = false;
        currentConfig.StatorCurrentLimit = curlimit;
        elemotor1.getConfigurator().apply(currentConfig);
        elemotor2.getConfigurator().apply(currentConfig);
        elemotor1.setNeutralMode(NeutralModeValue.Brake);
        elemotor2.setNeutralMode(NeutralModeValue.Brake);
        
        var elevatorslot0 = new Slot0Configs()
        .withKA(0)
        .withKD(0)
        .withKG(0)
        .withKI(0)
        .withKP(0.5)
        .withKS(0)
        .withKV(0);
        elemotor1.getConfigurator().apply(elevatorslot0);
        elemotor2.getConfigurator().apply(elevatorslot0);
        elemotor2.setControl(new Follower(elemotor1.getDeviceID(), true));
        this.positionSignal = elemotor1.getRotorPosition();
    }
    public void setV(double voltage)
    {
        elecontrol.Output = voltage;
        elemotor1.setControl(elecontrol);
        
    }
    private double getencoderval()
    {
        double value = positionSignal.getValueAsDouble();
        return value;
    } 
    public double getheight()
    {
        double value = positionSignal.getValueAsDouble();
        double height = value/4.5*dia;
        return height;
    }
    public void movetoheight(double setpoint)
    {
        double target = setpoint-this.getheight();
        SmartDashboard.putNumber("deltaheight", target);
        double rot = target/dia*4.5;
        SmartDashboard.putNumber("target rotation", rot);
        elemotor1.setControl(request.withPosition(rot));
    }
    public boolean reached(double pos)
    {
        if(Math.abs(pos-this.getheight())<=2)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    public void resetencoder()
    {
        this.elemotor1.setPosition(0);
    }
    @Override
    public void periodic()
    {
        BaseStatusSignal.refreshAll(positionSignal);
        SmartDashboard.putNumber("elevator rotation", this.getencoderval());
        SmartDashboard.putNumber("elevator height", this.getheight());
    }
}
