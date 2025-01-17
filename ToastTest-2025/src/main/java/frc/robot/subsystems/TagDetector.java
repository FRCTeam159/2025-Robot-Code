// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.Core;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;

public class TagDetector extends Thread {
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }

  private UsbCamera intakeCamera;
  public static double maxPoseError = 2;

  protected static CvSource ouputStream;
  protected AprilTagDetector wpi_detector;

  AprilTagPoseEstimator.Config wpi_poseEstConfig;
  AprilTagPoseEstimator wpi_pose_estimator;

  Drivetrain m_drivetrain;

  static boolean m_targeting = false;

  public static double min_decision_margin = 30; // reject tags less than this

  static int count = 0;
  private CvSink UsbCameraSink;
  private Mat mat;
  static int IMAGE_WIDTH = 640;
  static int IMAGE_HEIGHT = 480;
  public double hFOV = 40.107;
  public double aspect = ((double) IMAGE_WIDTH) / IMAGE_HEIGHT;
  public double vFOV = hFOV / aspect;
  public double cx = IMAGE_WIDTH / 2.0;
  public double cy = IMAGE_HEIGHT / 2.0;
  public double fx = cx / Math.tan(0.5 * Math.toRadians(hFOV));
  public double fy = cy / Math.tan(0.5 * Math.toRadians(vFOV));

  double targetSize = 0.1524;

  public static boolean showTags =  false;
  public static boolean autoselect =  false;

  public TagDetector(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  @Override
  public void run() {
    mat = new Mat();
    intakeCamera = CameraServer.startAutomaticCapture(0); // specs for Gazebo camera
    intakeCamera.setResolution(IMAGE_WIDTH, IMAGE_HEIGHT);
    intakeCamera.setFPS(25);
    UsbCameraSink = CameraServer.getVideo(intakeCamera);

    wpi_detector = new AprilTagDetector();
    try{
      wpi_detector.addFamily("tag16h5", 0);
      wpi_detector.addFamily("tag36h11", 0);
      System.out.println("Tag Families loaded");
    } catch (Exception ex){
      System.out.println("TagDetector exception:" + ex);
    }

    wpi_poseEstConfig = new AprilTagPoseEstimator.Config(targetSize, fx, fy, cx, cy);
    wpi_pose_estimator = new AprilTagPoseEstimator(wpi_poseEstConfig);

    ouputStream = CameraServer.putVideo("RobotCamera", IMAGE_WIDTH, IMAGE_HEIGHT);

    while (!Thread.interrupted()) {
      try {
        Thread.sleep(30);
        long tm = UsbCameraSink.grabFrame(mat);
        if (tm == 0) // bad frame
          continue;
        
        ouputStream.putFrame(mat);
      } catch (Exception ex) {
        System.out.println("TagDetector exception:" + ex);
      }
    }
  }

  
}
