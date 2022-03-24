package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;

//import org.jumprobotics.robot.drivers.*;

public class limelight extends SubsystemBase{

    NetworkTable table;
    NetworkTableEntry xOffset;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;
    double limelightX;
    double limelightY;
    double limelightArea;
    double targetExists;
    double distance;

    public static limelight instance;

    // inches
    double limelightHeightDifference;
    

    public limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        xOffset = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        limelightHeightDifference = Constants.HUBHEIGHT - Constants.LIMELIGHTHEIGHT;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        limelightX = xOffset.getDouble(0.0);
        limelightY = ty.getDouble(0.0);
        limelightArea = ta.getDouble(0.0);
        targetExists = ((tv.getDouble(0.0)==1) ? 1d : 0d);

        SmartDashboard.putNumber("Predicted Distance ", this.getDistance());
    }

    public static limelight getInstance(){
        if(instance == null){
          instance = new limelight();
        }
        return instance;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public double getDistance() {
        // 56 inch difference
        double newDistance = 1*limelightHeightDifference/Math.tan((limelightY+Constants.LIMELIGHTANGLE)*Math.PI/180);
        if(Math.abs(newDistance - 141.537457) < 0.1 ) {
            return distance;
        }
        distance = newDistance;
        return distance;
        //return 1*limelightHeightDifference/Math.tan((limelightY+Constants.LIMELIGHTANGLE)*Math.PI/180);
    }

    public double getXOffset() {
        return limelightX; // return the x-offset from the camera to reflective tape
    }
}
