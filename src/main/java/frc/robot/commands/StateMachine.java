package frc.robot.commands;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Macro;
import frc.robot.enums.StateEnum;
public class StateMachine {
    private Macro macro;
    public StateEnum curState = StateEnum.NONE;
    public StateMachine(Macro macro)
    {
        this.macro = macro;
    };
    public Command changestate(StateEnum state)
    {
        return new InstantCommand(() -> {
            curState = state;
        });
    }   
}
