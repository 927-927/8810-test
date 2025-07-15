package frc.robot.commands;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.enums.StateEnum;
import frc.robot.subsystems.*;
public class Macro {
    //teleop key binding
        //coral
        //L3 550
        //L4 1170
        //L2  deg 100 ele
        //L1 110 deg 500
        //algae
        //L2 85deg 430 ele
        //L3 85deg 830ele
        //score -30deg 1430ele
        //ground 126deg 133ele
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    public StateEnum curState = StateEnum.NONE;
    public Command coralL1, coralL2, coralL3, coralL4;
    public Command algaeground, algaeL2, algaeL3, processor, barge, homelarger30,homesmaller30,home,algaehold,algaehold1;
    public Command elepidtune;
    public Command autonscore;
    public Command loading;
    public Macro(ElevatorSubsystem elevatorSubsystem,ArmSubsystem armSubsystem,IntakeSubsystem intakeSubsystem)
    {
        this.elevator = elevatorSubsystem;
        this.arm = armSubsystem;
        this.intake = intakeSubsystem;

        coralL1 = Commands.sequence(
            arm.setangle(110),
            new WaitUntilCommand(() -> arm.getdegree() > 20),
            elevator.setheight(500)
        );

        coralL2 = Commands.sequence(
            arm.setangle(30),
            new WaitUntilCommand(() -> arm.getdegree() > 20),
            elevator.setheight(100)
        );

        coralL3 = Commands.sequence(
            arm.setangle(30),
            new WaitUntilCommand(() -> arm.getdegree() > 20),
            elevator.setheight(550)
        );

        coralL4 = Commands.sequence(
            arm.setangle(30),
            new WaitUntilCommand(() -> arm.getdegree() > 20),
            elevator.setheight(1170)
        );

        algaeground = Commands.sequence(
            arm.setangle(126),
            new WaitUntilCommand(() -> arm.getdegree() > 20),
            elevator.setheight(133)
        );

        algaeL2 = Commands.sequence(
            arm.setangle(85),
            new WaitUntilCommand(() -> arm.getdegree() > 20),
            elevator.setheight(430)
        );

        algaeL3 = Commands.sequence(
            arm.setangle(85),
            new WaitUntilCommand(() -> arm.getdegree() > 20),
            elevator.setheight(830)
        );

        processor = Commands.sequence(
            arm.setangle(85),
            new WaitUntilCommand(() -> arm.getdegree() > 20),
            elevator.setheight(0)
        );

        barge = Commands.sequence(
            elevator.setheight(1430),
            new WaitUntilCommand(() -> elevator.getheight() > 1000),
            arm.setangle(-30)
        );

        homelarger30 = Commands.sequence(
            elevator.setheight(250),
            new WaitUntilCommand(() -> elevator.getheight() > 150),
            arm.setangle(179),
            intake.intakedirect(-0.05)
        );

        homesmaller30 = Commands.sequence(
            arm.setangle(40),
            new WaitUntilCommand(() -> arm.getdegree() > 30),
            elevator.setheight(250),
            new WaitUntilCommand(() -> elevator.getheight() > 150),
            arm.setangle(179),
            intake.intakedirect(-0.05)
        );

        home = new ConditionalCommand(homelarger30, homesmaller30,() -> arm.getdegree()>30);

        algaehold = Commands.sequence(
            arm.setangle(30),
            elevator.setheight(30)
        );

        algaehold1 = Commands.sequence(
            arm.setangle(30),
            elevator.setheight(30)
        );

        elepidtune = Commands.sequence(
            elevator.setheight(0),
            new WaitUntilCommand(() -> elevator.getheight() < 100),
            arm.setangle(5)
        );

        autonscore = Commands.sequence(
            elevator.setheight(1200),
            new WaitUntilCommand(() -> elevator.getheight()>1150),
            arm.setangle(67),
            new WaitUntilCommand(() -> arm.getdegree()>63),
            intakeSubsystem.intakedirect(0.5)
        );



    }

    public Command changestate(StateEnum state) {
        return new InstantCommand(() -> {
            this.curState = state;
            SmartDashboard.putString("state", this.curState.toString());
        });
    }


    
}