// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class AimAndMove extends CommandBase {
  public AimAndMove() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // CAMERA HEIGHT + TARGET HEIGHT (CHIỀU CAO CAMERA + CHIỀU CAO VẬT THỂ)
    final double cameraHeightMeters = Units.inchesToMeters(Constants.CameraHeightInches);
    final double targetHeightMeters = Units.feetToMeters(Constants.TargetHeightFeet);
    
    //HORIZONTAL CAMERA'S ANGLE (GÓC NGANG CỦA CAMERA)
    final double cameraPitchRadians = Units.degreesToRadians(Constants.CameraPitchDegrees);

    //TARGET DISTANCE(KHOẢNG CÁCH CAM ĐẾN VẬT THỂ)
    final double targetPitchRadians = Units.degreesToRadians(Constants.TargetDisDegrees);

    PhotonCamera camera = new PhotonCamera("Tên Camera"); //Vào UI của Photon Vision và gõ đúng tên là được

    //PID CONSTANTS (CHỈNH TEST SỐ SAU)
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    XboxController xboxController = new XboxController(0);

    //Drive motors (Khai left, right motor trong Subsystem đã nhé)
    DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    double forwardSpeed;
    double rotationSpeed;

    if (xboxController.getAButton()) {
      //Vision-alignment mode 
      //Query the lastest result from PhotonVision
      var result = camera.getLatestResult();

      if (result.hasTargets()) {
        double range = 
                PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters,
                 targetHeightMeters, cameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));

        //RANGE GIVEN TO PID CONTROLLER
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
