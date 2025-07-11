package frc.robot.subsystems;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;

public class Candle extends SubsystemBase {
    CANdle candle;
    SolidColor color;

    public Candle(int deviceId) {
        candle = new CANdle(deviceId, TunerConstants.kCANBus);
    }

    public void ColorGenerate(int startIdx, int endIdx, String hexCode) {
        this.color = new SolidColor(startIdx, endIdx);
        Color8Bit hexColor = new Color8Bit(hexCode);
        RGBWColor colorRGBValue = new RGBWColor(hexColor);
        this.color.Color = colorRGBValue;
        this.color.withUpdateFreqHz(1000); 
        this.color.withColor(colorRGBValue);
    }

    public void setColor() {
        this.ColorGenerate(0, 30, "#87254a");
        candle.setControl(this.color);
    }
}
