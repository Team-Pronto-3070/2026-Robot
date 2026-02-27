package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class LEDSubsystem extends SubsystemBase{

    private final AddressableLED led;
    private AddressableLEDBuffer buffer;

    private LEDPattern pattern;

    //max brightness
    private final LEDPattern rainbowTemplate = LEDPattern.rainbow(255, 64);
    private final LEDPattern gradTemplate = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kCyan, Color.kBlack);

    //14 LEDs per meter 
    private static final Distance kLedSpacing = Meters.of(1 / 14.0);
    
    private Boolean hasLock = false;
    private Boolean isShooting = false;

    private final LEDPattern ScrollingRainbow = rainbowTemplate.scrollAtAbsoluteSpeed(MetersPerSecond.of(10), kLedSpacing);
    private final LEDPattern ScrollingGrad = gradTemplate.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5), kLedSpacing);

    // private final LEDPattern BreatheRainbow = rainbowTemplate.breathe(Seconds.of());
    //private final LEDPattern BreatheGrad = gradTemplate.breathe(Seconds.of(0.5));

    /*
    * Disabled - Cool pattern
    * Enabled
    *   - Turret has lock
    *       - Green
    *       Shooting
    *           - Pulsing/flowing green
    *   - Turret doesn't have lock
    *      - Cyan
    */


    public LEDSubsystem(){
        //PWM port 0 (must be PWM)
        led = new AddressableLED(0);

        buffer = new AddressableLEDBuffer(17);
        led.setLength(buffer.getLength());

        led.setData(buffer);
        led.start();

    }

    public void setLock(boolean lock){
        hasLock = lock;
    }

    public void setShooting(boolean shooting){
        isShooting = shooting;
    }

    public void setColor(Color c){
        pattern = LEDPattern.solid(c);
    }

    public void setPattern(LEDPattern p){
        pattern = p;
    }

    @Override
    public void periodic(){
        if (DriverStation.isEnabled()) {
            if (hasLock) {
                if (isShooting) {
                    setPattern(ScrollingGrad);
                } else {
                    setColor(Color.kCyan);
                }
            } else {
                setColor(Color.kOrange);
            }
        } else {
            setPattern(ScrollingRainbow);
        }

        pattern.applyTo(buffer);
        led.setData(buffer);
    }
}
