package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {
    
    private CommandXboxController driver;
    private CommandXboxController operator;

    public final DoubleSupplier drive_x;
    public final DoubleSupplier drive_y;
    public final DoubleSupplier drive_rot;

    public final Trigger gyroReset;
    
    public final DoubleSupplier processed_drive_x;
    public final DoubleSupplier processed_drive_y;
    public final DoubleSupplier processed_drive_rot;

    public final Trigger shoot;
    public final Trigger index;

    public OI(){
        driver = new CommandXboxController(Constants.OI.driverPort);
        operator = new CommandXboxController(Constants.OI.operatorPort);

        drive_x = () -> -driver.getLeftY();
        drive_y = () -> -driver.getLeftX();
        drive_rot = () -> -driver.getRightX();

        processed_drive_x = () -> MathUtil.copyDirectionPow(drive_x.getAsDouble(), 2);
        processed_drive_y = () ->  MathUtil.copyDirectionPow(drive_y.getAsDouble(), 2);
        processed_drive_rot = () ->  MathUtil.copyDirectionPow(drive_rot.getAsDouble(), 2);

        gyroReset = driver.back();

        index = driver.rightBumper();
        shoot = driver.a();
    }

}
