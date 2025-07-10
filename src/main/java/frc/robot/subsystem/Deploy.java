package frc.robot.subsystem;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.*;

public class Deploy extends SubsystemBase{
    private Servo deploy1 = new Servo(1);
    private Servo deploy2 = new Servo(2);
    public Deploy()
    {}
    private void release(double angle)
    {
        deploy1.setAngle(angle);
        deploy2.setAngle(angle);
    }
    public Command Release()
    {
        return this.runOnce(
            () -> release(90)
        );
    }
}
