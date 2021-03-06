// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Camera Constant (Vision)
    public static double timeConstant = 0;
    public static double period = 0;


    //AimAndMoveJavaConstant
    public static double CameraHeightInches = 0;
    public static double TargetHeightFeet = 8.66141732; //Không chỉnh lại vì đã bám sát theo Manual rồi
    public static double CameraPitchDegrees = 0; //Đo bằng điện thoại hay gì đấy tùy tính sau 
    public static double TargetPitchDegrees = 0; //Đo bằng Pipelines trong Photon UI


    //AimAndMove PID Constants (Linear + Angular kP , kD) + Kiếm trong PhotonUI + Test PID của bot
    public static double LINEAR_P = 0;
    public static double LINEAR_D = 0; 
    public static double ANGULAR_P = 0;
    public static double ANGULAR_D = 0;

    //Vision Subsystem 
    public static String camName = "Tên Camera"; // Xem trong UI của PhotonLib qua web
    public static double camDiagFOV = 0; // degrees
    public static double camPitch = 0; // degrees
    public static double camHeightOffGround = 0; // meters
    public static double maxLEDRange = 0; // meters
    public static int camResolutionWidth = 0; // pixels
    public static int camResolutionHeight = 0; // pixels
    public static double minTargetArea = 0; // square pixels
    public static Transform2d cameraToRobot; //I DUNNO THIS ONE

    // DRIVE TRAIN SIM CONSTANT
    //Linear Constant Values
    public static double kVLinear = 0; 
    public static double kALinear = 0;
    public static double kVAngular = 0;
    public static double kAAngular = 0;

    //Dimensional size of robots
    public static double driveMotor = 0;
    public static double gearing = 0;
    public static double jKgMetersSquared =0;
    public static double massKg = 0;
    public static double wheelRadiusMeters = 0;
    public static double trackWidthMeters = 0;
    public static SimVisionTarget kFarTarget;
    public static Matrix<N7, N1> measurementStdDevs; 
 


}
