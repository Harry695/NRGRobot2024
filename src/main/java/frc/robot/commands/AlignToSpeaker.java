/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AlignToSpeaker extends Command {
  Optional<AprilTagSubsystem> aprilTag;
  SwerveSubsystem drivetrain;
  Optional<PhotonTrackedTarget> optionalTagTarget;

  @RobotPreferencesValue
  public static RobotPreferences.DoubleValue KP =
      new RobotPreferences.DoubleValue("AprilTag", "Speaker Alignment KP", 1.0);

  PIDController angleController = new PIDController(KP.getValue(), 0, 0);

  /** Creates a new AlignToSpeaker. */
  public AlignToSpeaker(Subsystems subsystems) {
    aprilTag = subsystems.aprilTag;
    drivetrain = subsystems.drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(aprilTag.get(), drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    optionalTagTarget = aprilTag.get().getTarget(AprilTagSubsystem.getSpeakerCenterAprilTagID());

    if (!optionalTagTarget.isPresent()) {
      return;
    }

    double angleToTarget = Math.toRadians(-optionalTagTarget.get().getYaw());
    double rSpeed = angleController.calculate(angleToTarget);

    drivetrain.drive(0, 0, rSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleController.atSetpoint();
  }
}
