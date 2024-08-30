/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;

public class DriveUsingController extends Command {
  private static final double DEADBAND = 0.08;

  @RobotPreferencesValue(column = 0, row = 1)
  public static final RobotPreferences.DoubleValue AUTO_ORIENT_KP =
      new RobotPreferences.DoubleValue("Drive", "Auto Orient kP", 1.0);

  @RobotPreferencesValue(column = 1, row = 1)
  public static final RobotPreferences.DoubleValue AUTO_ORIENT_KI =
      new RobotPreferences.DoubleValue("Drive", "Auto Orient kI", 0);

  @RobotPreferencesValue(column = 2, row = 1)
  public static final RobotPreferences.DoubleValue AUTO_ORIENT_KD =
      new RobotPreferences.DoubleValue("Drive", "Auto Orient kD", 0);

  private final SwerveSubsystem drivetrain;
  private final CommandXboxController xboxController;
  private ProfiledPIDController controller;

  private double previousOrientation = 0;
  private double previousRInput = 0;
  private Timer timer;

  /** Creates a new DriveUsingController. */
  public DriveUsingController(Subsystems subsystems, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subsystems.drivetrain;
    this.xboxController = xboxController;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // We initialize the PID controller and PID constant in initialize. To make preferences changes
    // take effect immediately, press the "interrupt all" button.
    controller =
        new ProfiledPIDController(
            AUTO_ORIENT_KP.getValue(),
            AUTO_ORIENT_KI.getValue(),
            AUTO_ORIENT_KD.getValue(),
            SwerveSubsystem.getRotationalConstraints());
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setIZone(Math.toRadians(5));
    Rotation2d currentOrientation = drivetrain.getOrientation();
    controller.reset(currentOrientation.getRadians());

    timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rSpeed;
    double xSpeed = -xboxController.getLeftY();
    double ySpeed = -xboxController.getLeftX();
    double inputScalar = Math.max(1.0 - xboxController.getRightTriggerAxis(), 0.15);

    // Applies deadbands to x and y joystick values and multiples all
    // values with inputScalar which allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * inputScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * inputScalar;

    Optional<Rotation2d> targetOrientation = drivetrain.getTargetOrientation();

    double currentOrientation = drivetrain.getOrientation().getRadians();

    // get controller rotation input
    double rInput = -xboxController.getRightX();
    rInput = MathUtil.applyDeadband(rInput, DEADBAND) * inputScalar;

    // check if maintain orientation is needed
    if (targetOrientation.isEmpty() && rInput == 0) {
      // assign last orientation when input is not 0 to the target orientation
      if (previousRInput != 0) {
        // estimate the angle that the angular momentum will carry the robot after rotation stops
        // using kinematics
        // double measuredOmega =
        //     (currentOrientation - previousOrientation)
        //         / (Timer.getFPGATimestamp() - previousTimeStamp);
        // double expectedOrientation =
        //     currentOrientation
        //         + Math.signum(measuredOmega)
        //             * Math.pow(measuredOmega, 2)
        //             / (2
        //                 * SwerveSubsystem.getRotationalConstraints()
        //                     .maxAcceleration); // assumes acceleration is max

        timer.reset();
        timer.start();
      }
      // allows time for omega to go to 0, then set target orientation
      if (timer.get() > 0.5) {
        targetOrientation = Optional.of(Rotation2d.fromRadians(currentOrientation));
        timer.reset();
        timer.stop();
      }
    }

    if (targetOrientation
        .isPresent()) { // if there targetOrientation wasn't empty to begin with OR maintain
      // orientation is active
      double feedback =
          controller.calculate(currentOrientation, targetOrientation.get().getRadians());

      rSpeed =
          feedback
              + (controller.getSetpoint().velocity
                  / SwerveSubsystem.getRotationalConstraints().maxVelocity);
    } else { // if no targetOrientation, just use rotation stick input
      rSpeed = rInput;
    }

    previousRInput = rInput;
    previousOrientation = currentOrientation;

    drivetrain.drive(xSpeed, ySpeed, rSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
