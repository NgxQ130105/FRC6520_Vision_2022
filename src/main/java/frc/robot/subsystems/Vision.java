// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.photonvision.PhotonCamera; //NOTE: Before importing PhotonCamera install vendor libr: https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import frc.robot.Constants;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private FilterValues filtervalues;
  PhotonCamera camera;
  private LinearFilter xOffSetFilter;
  private LinearFilter yOffSetFilter; 
   
  public Vision() {
    camera = new PhotonCamera("Logitech C920"); 

    xOffSetFilter = LinearFilter.singlePoleIIR(Constants.timeConstant, Constants.period);
    yOffSetFilter = LinearFilter.singlePoleIIR(Constants.timeConstant, Constants.period);

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
