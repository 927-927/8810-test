// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmManual;
import frc.robot.commands.IntakeManual;
import frc.robot.subsystem.ArmSubsystem;
import frc.robot.subsystem.ElevatorSubsystem;
import frc.robot.subsystem.IntakeSubsystem;

public class RobotContainer {
  
  public final CANBus maincanbus = new CANBus("mainCAN");
  private final ArmSubsystem armSubsystem = new ArmSubsystem (maincanbus,0.0);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(maincanbus,5,0.0);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(maincanbus);
  public final CommandXboxController joystick = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
    CommandScheduler.getInstance().setDefaultCommand(elevatorSubsystem,elevatorSubsystem.elevatorPID());
    CommandScheduler.getInstance().setDefaultCommand(armSubsystem, armSubsystem.armPID());
    SignalLogger.setPath("/U/logs");
    SignalLogger.start();
  }
  private void configureBindings() 
  {
      joystick.a().whileTrue(elevatorSubsystem.setheight(100));
      joystick.b().whileTrue(armSubsystem.setangle(3));

  } 

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public ElevatorSubsystem getElevatorSubsystem() {
    return elevatorSubsystem;
  }
}

