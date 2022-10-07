
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import javax.swing.JFrame;

import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class Camera extends SubsystemBase {
  // Creates UsbCamera and MjpegServer [1] and connects them

  public CvSink cvSink;
  public CvSource outputStream;
  public Mat img;
    // Creates the CvSink and connects it to the UsbCamera

  public Camera() {
    CameraServer.startAutomaticCapture();
    cvSink = CameraServer.getVideo();
    outputStream = CameraServer.putVideo("Blur", 640, 480);
    HighGui.namedWindow("BlueBalls");
    HighGui.namedWindow("RedBalls");
    HighGui.waitKey();
    System.out.println("yes");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Mat orImage = Imgcodecs.imread("rage.jpg");
    List<Mat> colors = new ArrayList<Mat>();
    //cvSink.grabFrame(img);
    Core.split(orImage,colors);
    Mat sheesh[] = new Mat[colors.size()]; 
    
    colors.toArray(sheesh);
    Mat BlueMask = sheesh[0];
    Mat RedMask = sheesh[2];
    Imgproc.blur(BlueMask, BlueMask, new Size(3,3));
    Imgproc.blur(RedMask, RedMask, new Size(3,3));
    drawCrossHairs(BlueMask);
    drawCrossHairs(RedMask);
    HighGui.imshow("BlueBalls", applyBoundingBox(BlueMask));
    HighGui.imshow("RedBalls", applyBoundingBox(RedMask));
    
  }
  public static Mat drawCrossHairs(Mat targImg){
    targImg.put((int)(targImg.rows()/2), (int)(targImg.cols()/2), 255);
    for (int i = 0; i>20; i++){
        targImg.put((int)(targImg.rows()/2)+1, (int)targImg.cols()/2, 255);
        targImg.put((int)(targImg.rows()/2)-1, (int)targImg.cols()/2, 255);
        targImg.put((int)(targImg.rows()/2), (int)(targImg.cols()/2)+1, 255);
        targImg.put((int)(targImg.rows()/2), (int)(targImg.cols()/2)-1, 255);
    }
    return targImg;
  }
  public static Mat applyBoundingBox(Mat targImg){
    Mat sheesh = new Mat();
    Mat edgedUpMap = new Mat();
    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    Mat heirarchy = new Mat();
    Imgproc.threshold(targImg, sheesh, 120, 255, Imgproc.THRESH_BINARY);
    Imgproc.findContours(sheesh, contours,heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
    Imgproc.drawContours(edgedUpMap, contours, -1, new Scalar(0,255,0));
    return edgedUpMap;
  }
}
//action: the avengers, the matrix
// animated =  none
//comedy: inside out