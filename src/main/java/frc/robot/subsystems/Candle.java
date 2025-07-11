package frc.robot.subsystems;

import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;


public class Candle extends SubsystemBase {
    CANdle candle;

    public Candle(int deviceId) {
        candle = new CANdle(deviceId, TunerConstants.kCANBus);
    }

    public SolidColor ColorGenerate(int startIdx, int endIdx, String hexCode) {
        SolidColor color = new SolidColor(startIdx, endIdx);
        Color8Bit hexColor = new Color8Bit(hexCode);
        RGBWColor colorRGBValue = new RGBWColor(hexColor);
        color.Color = colorRGBValue;
        color.withUpdateFreqHz(1000); 
        color.withColor(colorRGBValue);
        return color;
    }

    // public void ColorFlowGenerate(int startIdx, int endIdx, String hexCode, int slot) {

    //     Color8Bit hexColor = new Color8Bit(hexCode);
    //     RGBWColor colorRGBValue = new RGBWColor(hexColor);
        
    // }

    public void setColor(SolidColor color) {
        candle.setControl(color);
    }
}
