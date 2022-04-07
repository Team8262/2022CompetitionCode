// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Rect;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallVision extends SubsystemBase {
  private UsbCamera yee;
  Mat img;
  /** Creates a new BallVision. */
  public BallVision() {
    yee = CameraServer.startAutomaticCapture(0);
    SmartDashboard.putString("HELP ", "sd");
    img = new Mat();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    CvSink cv = CameraServer.getVideo(yee);
    cv.grabFrame(img);
    img = img.submat(new Rect(Constants.cameraROI[0], Constants.cameraROI[1]));
    int rows = img.rows();
    int cols = img.cols();

    double red = 0;
    double blue = 0;
    double green = 0;
    for(int i=0;i<rows;i++){
        for(int j=0;j<cols;j++){
            double[] arr = img.get(i,j); //BGR???
            blue += arr[0];
            green += arr[1];
            red += arr[2];
        }
    }
    SmartDashboard.putNumberArray("RGB", new double[]{red, green, blue});
  }
}
