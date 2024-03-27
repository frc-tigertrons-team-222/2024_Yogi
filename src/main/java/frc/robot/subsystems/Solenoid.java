package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;





public class Solenoid extends SubsystemBase {

    public PowerDistribution SolenoidSwitch = new PowerDistribution(22,PowerDistribution.ModuleType.kRev );
    public boolean switchState = false;

    public Command switchOff() {
        switchState = false;
        return runOnce(() -> SolenoidSwitch.setSwitchableChannel(false));
    }

    public Command switchOn() {
        switchState = true;
        return runOnce(() -> SolenoidSwitch.setSwitchableChannel(true));
    }

    public Command switchToggle() {
        return new RunCommand(() -> {
          switchToggleMethod();
        });
    }
    public void switchToggleMethod() {
       
            if (switchState) {
                SolenoidSwitch.setSwitchableChannel(false);
                switchState=false;
            } else {
                SolenoidSwitch.setSwitchableChannel(true);
                switchState=true;
            }
    
    }
    
    public void periodic(){
    }
}
