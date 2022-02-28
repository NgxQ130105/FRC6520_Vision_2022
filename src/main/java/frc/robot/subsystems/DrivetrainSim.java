// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.SimVisionSystem;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSim extends SubsystemBase {
  // Insert Sensors

  // Insert Motor Controllers


  // Simulation Physics
  // Configure these to match your drivetrain's physical dimensions
  // and characterization results. (Linear System usinf SysId)
  LinearSystem<N2, N2, N2> drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(kVLinear, kALinear, kVAngular, kAAngular);
  DifferentialDrivetrainSim drivetrainSimulator = new DifferentialDrivetrainSim(driveMotor, gearing, jKgMetersSquared, massKg, wheelRadiusMeters, trackWidthMeters, measurementStdDevs);

  //Simulated Vision System
  //Configure to match Cam + LED + pipeline
  SimVisionSystem simVision = new SimVisionSystem(Constants.camName, Constants.camDiagFOV, Constants.camPitch, Constants.cameraToRobot, 
  Constants.camHeightOffGround, Constants.maxLEDRange, Constants.camResolutionWidth, Constants.camResolutionHeight, Constants.minTargetArea);

  public DrivetrainSim() {
    simVision.addSimVisionTarget(Constants.kFarTarget);
  }
  
  public void update() {
    double leftMotorCmd = 0;
    double rightMotorCmd = 0;

    if (DriverStation.isEnabled() && !RobotController.isBrownedOut()) {
                  // If the motor controllers are enabled...
            // Roughly model the effect of leader and follower motor pushing on the same
            // gearbox.
            // Note if the software is incorrect and drives them against each other, speed
            // will be
            // roughly matching the physical situation, but current draw will _not_ be
            // accurate.
        leftMotorCmd = (leftMaster.getSpeed() + lefFollower.getSpeed()) / 2.0;
        rightMotorCmd = (rightMaster.getSpeed() + rightFollower.getSpeed()) / 2.0;
    }

    //Update the physics simulation

  }


  }

