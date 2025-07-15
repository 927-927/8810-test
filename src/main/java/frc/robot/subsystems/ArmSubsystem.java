package frc.robot.subsystems;

import java.security.AlgorithmParameterGenerator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private final TalonFX armmotor;
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    private PositionVoltage request = new PositionVoltage(0).withSlot(0).withOverrideBrakeDurNeutral(true);
    final DutyCycleOut armcontrol = new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true);
    private double setpoint;
    private BaseStatusSignal statorcurrent;
    public double speed = 1;
    private final BaseStatusSignal positionSignal;
    LinearFilter filter = LinearFilter.movingAverage(4);
    private MotorOutputConfigs armconfig = new MotorOutputConfigs();
    public ArmSubsystem(CANBus canBus,double setpoint) {
        this.armmotor = new TalonFX(20,canBus);
        armconfig.Inverted = InvertedValue.Clockwise_Positive;
        armmotor.getConfigurator().apply(armconfig);
        this.positionSignal = armmotor.getRotorPosition();
        var armslot0 = new Slot0Configs();
        armslot0.kS = 0.0;
        armslot0.kV = 0.0;
        armslot0.kA = 0.0;
        armslot0.kP = 0.6;
        armslot0.kI = 0.1;
        armslot0.kD = 0.0;
        statorcurrent = armmotor.getStatorCurrent();
        armmotor.getConfigurator().apply(armslot0);
        armmotor.setControl(armcontrol);
        armmotor.setNeutralMode(NeutralModeValue.Brake);
        this.setpoint = setpoint;
        armmotor.setPosition(0);
    }
    //encoder readings 
    private double getencoderval()
    {
        double value = positionSignal.getValueAsDouble();
        return value;
    } 
    public double getdegree()
    {
        double value = positionSignal.getValueAsDouble();
        double degree = value/58.33*360;
        return degree;
    }




    //set voltage
    public void setV(double voltage)
    {
        armcontrol.Output = voltage;
        armmotor.setControl(armcontrol);
    }
    //arm manual iline command
    public Command armmanual(double voltage)
    {
        return this.startEnd(
            () -> setV(voltage),
            () -> setV(0.0)
            );
    }




    //pid setcontrol
    public void movetoheight()
    {
        double rot = (this.setpoint)*(58.33)/360;
        SmartDashboard.putNumber("arm setpoint", this.setpoint);
        armmotor.setControl(request.withPosition(rot));
    }
    //pid height setting
    public Command setangle(double angle)
    {
        return this.runOnce(() -> {
            this.setpoint = angle;
            movetoheight();
        });
    }
    public void seta(double angle)
    {
        this.setpoint = angle;
    }
    //pid inline command
    public Command armPID()
    {
        return this.runOnce(() -> movetoheight());
    }



    public Command changespeed()
    {
        return new InstantCommand(() -> {
        if (speed == 1) {
            speed = 0.3;
        } else {
            speed = 1;
        }
        });
    }

    public boolean state()
    {
        if(this.getdegree()>=30)
        {
            return true;
        }
        else{
            return false;
        }
    }


    //periodic
    @Override
    public void periodic()
    {
        BaseStatusSignal.refreshAll(positionSignal,statorcurrent);
        SmartDashboard.putNumber("arm angle", this.getdegree());
        SmartDashboard.putNumber("speed", speed);
        SmartDashboard.putNumber("arm stator current", filter.calculate(statorcurrent.getValueAsDouble()));
    }
}

