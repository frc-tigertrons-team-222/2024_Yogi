package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-tiger");


    public LimelightSubsystem(){
    }

    // Double tv = table.getEntry("tv").getDouble(0.0);
    // Double tx = table.getEntry("tx").getDouble(0.0);
    // Double ty = table.getEntry("ty").getDouble(0.0);
    // Double ta = table.getEntry("ta").getDouble(0.0);

    // table.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);

    public double hasTarget(){
        return table.getEntry("tv").getDouble(0.0);
    }

    public double getXPosition(){
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getZPosition(){
        return table.getEntry("tz").getDouble(0.0);
    }

    public double getYPosition(){
        return table.getEntry("ty").getDouble(0.0);
    }

    public double getTargetArea(){
        return table.getEntry("ta").getDouble(0.0);
    }

    public double[] getTargetID(){
        return table.getEntry("tid").getDoubleArray(new double[6]);
    }

    public double[] getTarget3DPos(){
        double[] posArray = table.getEntry("campose").getDoubleArray(new double[6]);
        System.out.println("campose: " + posArray[2]);
        return table.getEntry("campose").getDoubleArray(new double[6]);
    }

    public double[] getTarget3DPosBlue(){
        return table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    public double[] getTarget3DPosRed(){
        double[] posArray = table.getEntry("botpose_wpired").getDoubleArray(new double[6]);
        
        System.out.println("botposered: " + posArray[2]);
        return posArray;
    }

    public double[] getTargetPoseRobotSpace(){
        double[] posArray = table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
        
        System.out.println("targetposerobotspace: " + posArray[3]);
        return posArray;
    }

    @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Has Target", table.getEntry("tv").getDouble(0.0));
    SmartDashboard.putNumber("X Position", table.getEntry("tx").getDouble(0.0));
    SmartDashboard.putNumber("Z Position", table.getEntry("tz").getDouble(0.0));
    SmartDashboard.putNumber("Y Position", table.getEntry("ty").getDouble(0.0));
    SmartDashboard.putNumber("Target Area", table.getEntry("ta").getDouble(0.0));
    SmartDashboard.putNumberArray("Target IDs", table.getEntry("tid").getDoubleArray(new double[6]));
    SmartDashboard.putNumberArray("Cam Pose", table.getEntry("campose").getDoubleArray(new double[6]));
    SmartDashboard.putNumberArray("Target Pose", table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]));
  }
}
