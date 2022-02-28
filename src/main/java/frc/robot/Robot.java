// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
      //High goal target height above ground (Độ cao của vật cần xđịnh so mặt đất)
      public static final double targetHeightMeters = Units.feetToMeters(Constants.TargetHeightFeet);
      
      //Camera Mounting (CamHeight - chiều cao từ mặt đất lên tới cam, Pitch - Angle "up" from horizontal)
      public static final double cameraHeightMeters = Units.inchesToMeters(Constants.CameraHeightInches);
      public static final double cameraPitchRadians = Units.degreesToRadians(Constants.CameraPitchDegrees); 
  
      // How far from the target we want to be (Khoảng cách từ lens tới băng phản quang hoặc vật thể)
      final double targetPitchRadians = Units.degreesToRadians(Constants.TargetPitchDegrees);
  
      PhotonCamera camera = new PhotonCamera("Tên Camera"); //Vào UI của Photon Vision (localhost) và gõ đúng tên là được
  
      //PID (Linear thì test có số liệu r bỏ vào còn Angular thì căn trong PhotonUI )
      PIDController forwardController = new PIDController(Constants.LINEAR_P, 0, Constants.LINEAR_D);
      PIDController turnController = new PIDController(Constants.ANGULAR_P, 0, Constants.ANGULAR_D);
      XboxController xboxController = new XboxController(0);
  
      //Drive motors (Khai left, right motor trong Subsystem đã nhé)
      DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);
  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {m_autonomousCommand.schedule();}}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {if (m_autonomousCommand != null) {m_autonomousCommand.cancel();}}

  @Override
  public void teleopPeriodic() {
    double forwardSpeed;
    double rotationSpeed;

    if (xboxController.getAButton()) { //Chỉnh button sau cũng được nhé
      //Vision-alignment mode 
      var result = camera.getLatestResult(); //Query the lastest result from PhotonVision
      if (result.hasTargets()) {
        double range = 
                PhotonUtils.calculateDistanceToTargetMeters(cameraHeightMeters,
                 targetHeightMeters, cameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));

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
  }
}