// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;


public class ArmManual extends Command {

  private final ArmSubsystem armSubsystem;
  private final double voltage;

  public ArmManual(ArmSubsystem armSubsystem,double voltage) {
    this.armSubsystem = armSubsystem;
    this.voltage = voltage;
    addRequirements(armSubsystem);
  }

 
  @Override
  public void initialize() {armSubsystem.setV(0.00);}

 
  @Override
  public void execute() 
  {
    armSubsystem.setV(voltage);
  }

 
  @Override
  public void end(boolean interrupted) 
  {
    armSubsystem.setV(0.00);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
