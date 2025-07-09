// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmManual;
import frc.robot.commands.ElevatorManual;
import frc.robot.commands.ElevatorPIDtest;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.ElevatorSubsystem;

public class RobotContainer {
  
  public final CANBus maincanbus = new CANBus("mainCAN");
  private final ArmSubsystem armSubsystem = new ArmSubsystem (maincanbus);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(maincanbus,5);
  public final Joystick joystick1 = new Joystick(3);  
  public RobotContainer() {
    configureBindings();
    SignalLogger.setPath("/U/logs");
    SignalLogger.start();
  }
  private void configureBindings() 
  {
    new Trigger(() -> joystick1.getRawButton(3)).whileTrue(new ElevatorPIDtest(elevatorSubsystem, 100));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
