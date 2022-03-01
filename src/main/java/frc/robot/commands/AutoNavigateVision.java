// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

//Importing Libraries
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.Vision.*;
import static frc.robot.RobotContainer.*;

public class AutoNavigateVision extends CommandBase {
  
  public AutoNavigateVision() {}
    //PID (Linear thì test có số liệu r bỏ vào còn Angular thì căn trong PhotonUI )
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    //Drive motors (Khai left, right motor trong Subsystem đã nhé)
    DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double forwardSpeed;
    double rotationSpeed;

    //Vision-alignment mode 
    var result = camera.getLatestResult(); //Query the lastest result from PhotonVision
    if (result.hasTargets()) {
      double range = 
              PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters,
              targetHeightMetters, cameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));

        //Range given to PID Controller
        // -1.0 required to ensure positive PID controller effort _increases_ range
      forwardSpeed = -forwardController.calculate(range, targetPitchRadians);

        // Also calculate angular power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
    } else {
        //IF NO TARGET THEN STOP
      forwardSpeed = 0;
      rotationSpeed = 0;
    }
      //USE FOWARD SPEED + TURN SPEED TO DRIVE DRIVETRAIN
      drive.arcadeDrive(forwardSpeed, rotationSpeed);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
