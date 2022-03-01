// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.PhotonCamera; //NOTE: Before importing PhotonCamera install vendor libr: https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import static frc.robot.Constants.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
      //Tên Camera lấy từ PhotonVision UI ra      
      public static PhotonCamera camera = new PhotonCamera(camName);


      //High goal target height above ground (Độ cao của vật cần xđịnh so mặt đất)
      public static final double targetHeightMetters = Units.feetToMeters(TargetHeightFeet);
      
      //Camera Mounting (CamHeight - chiều cao từ mặt đất lên tới cam, Pitch - Angle "up" from horizontal)
      public static final double cameraHeightMeters = Units.inchesToMeters(CameraHeightInches);
      public static final double cameraPitchRadians = Units.degreesToRadians(CameraPitchDegrees); 
        
      // How far from the target we want to be (Khoảng cách từ lens tới băng phản quang hoặc vật thể)
      public static final double targetPitchRadians = Units.degreesToRadians(TargetPitchDegrees);
        
        
      //PID (Linear thì test có số liệu r bỏ vào còn Angular thì căn trong PhotonUI )
      PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
      PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
      XboxController xboxController = new XboxController(0);


  private FilterValues filtervalues;
  private LinearFilter xOffSetFilter;
  private LinearFilter yOffSetFilter; 
  

  public Vision() {
    xOffSetFilter = LinearFilter.singlePoleIIR(timeConstant, period);
    yOffSetFilter = LinearFilter.singlePoleIIR(timeConstant, period);

  }

  //DriverMode (True/False to enable DriverMode) 
  public void setDriverMode(boolean DriverMode) {
    camera.setDriverMode(DriverMode);
  }

  //Turns on LED if Coprocessor supports
  public void setLED(boolean LED) {
    camera.setLED(LED ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonTrackedTarget target = camera.getLatestResult().hasTargets() ? camera.getLatestResult().getBestTarget()
      : new PhotonTrackedTarget(0, 0, 0, 0, new Transform2d(), new ArrayList<TargetCorner>());
    this.filtervalues = new FilterValues(xOffSetFilter.calculate(target.getYaw()), yOffSetFilter.calculate(target.getPitch()));
    SmartDashboard.putNumber("x offset", this.filtervalues.getFilteredXOffset());
    SmartDashboard.putNumber("y offset", this.filtervalues.getFilteredYOffset());
  }


  public FilterValues getFilteredOffset() {
    return this.filtervalues;
  }

  public class FilterValues {
    private double filteredXOffset;
    private double filteredYOffset;

    public FilterValues(double filteredXOffset, double filteredYOffset){
      this.filteredXOffset = filteredXOffset;
      this.filteredYOffset = filteredYOffset;
    }
    public double getFilteredXOffset(){
      return this.filteredXOffset;
    }
  
    public double getFilteredYOffset(){
      return this.filteredYOffset;
    }


  }

}
