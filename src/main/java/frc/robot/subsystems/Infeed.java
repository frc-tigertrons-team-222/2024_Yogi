package frc.robot.subsystems;

import frc.robot.subsystems.WristInfeed;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GroundInfeed;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class Infeed {
    public WristInfeed m_WristInfeed = new WristInfeed();
    public GroundInfeed m_GroundInfeed = new GroundInfeed();
    public DigitalInput limitSwitch0 = new DigitalInput(0);
    public boolean switchResult = false;


    public Infeed() {
        SmartDashboard.putBoolean("Limit Switch0", switchResult);
    };

    public Command Intake(double intakeSpeed) {
        //0.718 IS STILL A SAFE POSITION TO PICK UP NOTES
        return new ParallelCommandGroup(m_WristInfeed.ContinuousIntake(intakeSpeed),m_GroundInfeed.ContinuousIntake(intakeSpeed))
                        .until(() -> !this.getResult())
                        //.andThen(m_GroundInfeed.StopInfeed())
                        .andThen(m_WristInfeed.PIDStop().withTimeout(2))
                        .andThen(m_WristInfeed.StopInfeed());
        // return m_WristInfeed.ContinuousIntake().deadlineWith(m_GroundInfeed.ContinuousIntake())
        //         .until(() -> !limitSwitch.get()).andThen(m_WristInfeed.PIDStop());
    }

      public Command Outtake() {
        //0.718 IS STILL A SAFE POSITION TO PICK UP NOTES
        return new ParallelCommandGroup(m_WristInfeed.Outtake(),m_GroundInfeed.Outtake());
                       
        // return m_WristInfeed.ContinuousIntake().deadlineWith(m_GroundInfeed.ContinuousIntake())
        //         .until(() -> !limitSwitch.get()).andThen(m_WristInfeed.PIDStop());
    }



    public Command StopInfeed() {
        return m_WristInfeed.StopInfeed().andThen(m_GroundInfeed.StopInfeed());
    }

    public Command GoOutfeed() {
        return m_WristInfeed.Intake(1);
    }

    public boolean getResult() {
        return switchResult;
    }

    public void periodic(){
        switchResult = limitSwitch0.get();
        SmartDashboard.putNumber("TEST NUMBER", 2.5);
        SmartDashboard.putBoolean("Limit Switch0", switchResult);
       

    }

}
