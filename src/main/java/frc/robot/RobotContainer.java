// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.Thread.State;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import  frc.robot.commands.Macro;
import frc.robot.enums.StateEnum;
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final ArmSubsystem armSubsystem = new ArmSubsystem(TunerConstants.kCANBus,30.0);
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(TunerConstants.kCANBus, 5,0.0);
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(TunerConstants.kCANBus);
    private final Candle rGBCandle = new Candle(0);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Macro macro = new Macro(elevatorSubsystem,armSubsystem,intakeSubsystem);


    //
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        Map<String, Command> commands = new HashMap<>();
        commands.put("score", macro.autonscore);
        commands.put("home", macro.home);
        commands.put("expand", macro.holdingposition);
        commands.put("flip", macro.score);
        NamedCommands.registerCommands(commands);
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        // Warmup PathPlanner to avoid Java pauses
        // NamedCommands.registerCommand("score", macro.algaeL3);
        FollowPathCommand.warmupCommand().schedule();
        CommandScheduler.getInstance().setDefaultCommand(elevatorSubsystem,elevatorSubsystem.elevatorPID());
        CommandScheduler.getInstance().setDefaultCommand(armSubsystem, armSubsystem.armPID());
        SignalLogger.setPath("/U/logs");
        SignalLogger.start();
    }

    private void configureBindings() {
        
        // joystick.a().onTrue(armSubsystem.setangle(100).andThen(new WaitUntilCommand(() -> armSubsystem.getdegree()>60)).andThen(elevatorSubsystem.setheight(500)));
        joystick.a().onTrue(new SelectCommand<StateEnum>(
            Map.of(
            StateEnum.NONE, macro.algaeground.andThen(macro.changestate(StateEnum.ALGAE)),
            StateEnum.ALGAE, macro.processor,
            StateEnum.CORAL, macro.coralL1
            ),
            () -> macro.curState
            ));
        joystick.b().onTrue(new SelectCommand<StateEnum>(
            Map.of(
            StateEnum.NONE, macro.algaeL2.andThen(macro.changestate(StateEnum.ALGAE)),
            StateEnum.ALGAE, macro.algaehold,
            StateEnum.CORAL, macro.coralL2
            ),
            () -> macro.curState
            ));
        joystick.x().onTrue(new SelectCommand<StateEnum>(
            Map.of(
            StateEnum.NONE, macro.algaeL3.andThen(macro.changestate(StateEnum.ALGAE)),
            StateEnum.ALGAE, macro.algaehold1,
            StateEnum.CORAL, macro.coralL3
            ),
            () -> macro.curState
            ));
        joystick.y().onTrue(new SelectCommand<StateEnum>(
            Map.of(
            StateEnum.NONE, Commands.none(),
            StateEnum.ALGAE, macro.barge,
            StateEnum.CORAL, macro.coralL4
            ),
            () -> macro.curState
            ));
        joystick.povLeft().whileTrue(intakeSubsystem.intakeManual(-0.6));
        joystick.povRight().whileTrue(intakeSubsystem.intakeManual(1.0));
        joystick.povDown().onTrue(new SelectCommand<StateEnum>(
            Map.of(
                StateEnum.NONE, macro.loading.andThen(macro.changestate(StateEnum.CORAL)),
                StateEnum.ALGAE, Commands.none(),
                StateEnum.CORAL, Commands.none()
                ),
                 ()->macro.curState
                ));
        joystick.leftTrigger().onTrue(macro.changestate(StateEnum.NONE).andThen(macro.home));
        joystick.rightTrigger().onTrue(new SelectCommand<StateEnum>(
            Map.of(
                StateEnum.NONE, Commands.none(),
                StateEnum.ALGAE, Commands.none(),
                StateEnum.CORAL, armSubsystem.setangle(70)
                ),
                 ()->macro.curState
                )).onFalse(armSubsystem.setangle(30));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-(joystick.getLeftY()*(Math.abs(elevatorSubsystem.speed))) * (Math.abs(MaxSpeed))) // Drive forward with negative Y (forward)
                    .withVelocityY(-(joystick.getLeftX()*(Math.abs(elevatorSubsystem.speed)) * (Math.abs(MaxSpeed)))) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //***** */
        // joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.rightBumper().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press



        //***** */
        joystick.button(7).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public Candle getRGBSubsystem() {
        return rGBCandle;
    }
}
