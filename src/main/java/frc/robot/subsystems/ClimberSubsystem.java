package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.Climbstate;

public class ClimberSubsystem extends SubsystemBase{
    private final TalonFX roller;
    private final TalonFX climber;
    private final double climbingV = 0.5;
    private Integer counter = 0;
    public Climbstate curstate = Climbstate.bounce;
    Timer climb = new Timer();
    private double setpoint;
    private final DutyCycleOut climbcontrol = new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true);
    private final DutyCycleOut rollercontrol = new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true);
    private PositionVoltage climbrequest = new PositionVoltage(0).withSlot(0).withOverrideBrakeDurNeutral(true);
    public ClimberSubsystem(CANBus CAN,double setpoint)
    {
        roller = new TalonFX(22,CAN);
        climber = new TalonFX(23,CAN);
        var climbslot0 = new Slot0Configs()
        .withKA(0)
        .withKD(0)
        .withKG(0)
        .withKI(0)
        .withKP(0.51)
        .withKS(0)
        .withKV(0);
        this.setpoint = setpoint;
        climber.getConfigurator().apply(climbslot0);
    }

    private void climbsetV(double voltage)
    {
        climbcontrol.Output = voltage;
        climber.setControl(climbrequest);
    }

    private void rollersetV(double voltage)
    {
        rollercontrol.Output = voltage;
        roller.setControl(rollercontrol);
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

    //pid setcontrol
    public void movetounits()
    {
        SmartDashboard.putNumber("climber units", this.setpoint);
        climber.setControl(climbrequest.withPosition(setpoint));
    }
    public Command setunit(double unit)
    {
        return this.runOnce(() -> {
            this.setpoint = unit;
            movetounits();
        });
    }
    //pid inline command
    public Command climberPID()
    {
        return this.runOnce(() -> movetounits());
    }
    public Command one_click()
    {
        if(this.counter == 2)
        {
            return setunit(100).andThen(this.runOnce(() -> {
                this.counter = 0;
                rollersetV(1.0);
                this.curstate = Climbstate.climb;
            })).andThen(climberPID()); 
        }
        else if(this.counter == 1)
        {
            return this.runOnce(() -> {
                this.counter++;
            });
        }
        else{
            return this.runOnce(
                () -> {
                    this.counter++;
                    climb.restart();
                }
            );
        }
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("timer", climb.getTimestamp());
        if(climb.hasElapsed(1))
        {
            this.counter = 0;
            climb.stop();
        }
    }
}
