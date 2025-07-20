package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private final TalonFX elemotor1;
    private final TalonFX elemotor2;
    private final double dia = 38.2*Math.PI;
    private DutyCycleOut elecontrol = new DutyCycleOut(0.0).withOverrideBrakeDurNeutral(true);
    private PositionVoltage request = new PositionVoltage(0).withSlot(0).withOverrideBrakeDurNeutral(true);
    private final BaseStatusSignal positionSignal;
    private double setpoint;
    public double speed = 1;
    
    public ElevatorSubsystem(CANBus CAN,double curlimit,double setpoint)
    {
        // motor definition
        this.elemotor1 = new TalonFX(8,CAN);
        this.elemotor2 = new TalonFX(9,CAN);
        this.setpoint = setpoint;
        this.positionSignal = elemotor1.getRotorPosition();
        //break mode
        elemotor1.setNeutralMode(NeutralModeValue.Brake);
        elemotor2.setNeutralMode(NeutralModeValue.Brake);
        // pid constants
        var elevatorslot0 = new Slot0Configs()
        .withKA(0)
        .withKD(0)
        .withKG(0)
        .withKI(0.06)
        .withKP(0.51)
        .withKS(0)
        .withKV(0);
        elemotor1.getConfigurator().apply(elevatorslot0);
        elemotor2.getConfigurator().apply(elevatorslot0);
        //foller talon
        elemotor2.setControl(new Follower(elemotor1.getDeviceID(), true));
        this.elemotor1.setPosition(0);
    } 
    //encoder readings
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




    //reset condition
    public void resetencoder()
    {
        this.elemotor1.setPosition(0);
    }




    //motor voltage
    public void setV(double voltage)
    {
        elecontrol.Output = voltage;
        elemotor1.setControl(elecontrol);
        
    }
    //manual inline command
    public Command elevatormanual(double voltage)
    {
        return this.startEnd(
        () -> setV(voltage),
        () -> setV(0));
        
    }




    //pid setcontrol
    public void movetoheight()
    {
        double rot = this.setpoint/dia*4.5;
        SmartDashboard.putNumber("elevator setpoint", this.setpoint);
        elemotor1.setControl(request.withPosition(rot));
    }
    //pid height setting
    public Command setheight(double height)
    {
        return this.runOnce(() -> {
            this.setpoint = height;
            movetoheight();
        });
    }
    
    //pid inline command
    public Command elevatorPID()
    {
        return this.runOnce(() -> movetoheight());
    }







    //periodic
    @Override
    public void periodic()
    {
        BaseStatusSignal.refreshAll(positionSignal);
        if(this.getheight()>=510)
        {
            this.speed = 0.3;
        }
        else{
            this.speed = 1;
        }
        SmartDashboard.putNumber("elevator rotation", this.getencoderval());
        SmartDashboard.putNumber("elevator height", this.getheight());
    }
}
