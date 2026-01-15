package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class OI {
    
    private CommandXboxController driver;
    private CommandXboxController operator;

    public OI(){
        driver = new CommandXboxController(Constants.OI.driverPort);
        operator = new CommandXboxController(Constants.OI.operatorPort);
    }

}
