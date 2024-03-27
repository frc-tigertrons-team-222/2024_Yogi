package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.WristInfeed;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GroundInfeed;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class Infeed {
    public WristInfeed m_WristInfeed = new WristInfeed();
    public GroundInfeed m_GroundInfeed = new GroundInfeed();
    public DigitalInput limitSwitch0 = new DigitalInput(0);
    public boolean switchResult = false;

    public Infeed() {
        SmartDashboard.putBoolean("Limit Switch0", switchResult);
        Shuffleboard.getTab("limitswitchgraph").addNumber("limitswitch",(()->limitSwitch0.get()?1d:0d)); // 
    };

    public Command Intake(double intakeSpeed) {
        //0.718 IS STILL A SAFE POSITION TO PICK UP NOTES
        return new ParallelCommandGroup(m_WristInfeed.ContinuousIntake(intakeSpeed),m_GroundInfeed.ContinuousIntake(intakeSpeed)) //I thought we switched to sequential?
                        .until(() -> !limitSwitch0.get())
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
    public Command WristOuttake() {
        //0.718 IS STILL A SAFE POSITION TO PICK UP NOTES
        return m_WristInfeed.Outtake();
    
    }


    public Command StopInfeed() {
        return m_WristInfeed.StopInfeed().andThen(m_GroundInfeed.StopInfeed());
    }

    public Command GoOutfeed() {
        return m_WristInfeed.Intake(1);
    }

    public boolean getResult() {
        return limitSwitch0.get();
    }

    public void periodic(){

    }

}
