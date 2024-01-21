// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.RobotConstants;

/**
 * This subsystem is responsible for getting target information from
 * PhotonVision.
 */

@RobotPreferencesLayout(groupName = "AprilTag", row = 1, column = 4, width = 2, height = 1)
public class AprilTagSubsystem extends PhotonVisionSubsystemBase {

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue enableTab = new RobotPreferences.BooleanValue(
      "AprilTag", "Enable Tab", false);

  private DoubleLogEntry targetXLogger = new DoubleLogEntry(DataLogManager.getLog(), "AprilTag/Target X");
  private DoubleLogEntry targetYLogger = new DoubleLogEntry(DataLogManager.getLog(), "AprilTag/Target Y");
  private DoubleLogEntry targetAngleLogger = new DoubleLogEntry(DataLogManager.getLog(), "AprilTag/Target Angle");
  private AprilTagFieldLayout tagLayout;
  private SendableChooser<Integer> aprilTagIdChooser = new SendableChooser<Integer>();
  private AprilTagFieldLayout aprilTagLayout;

  /** Creates a new PhotonVisionSubsystem. */
  public AprilTagSubsystem() {
    super("948Mono001", RobotConstants.APRILTAG_CAMERA_TO_ROBOT);

    for (int i = 1; i <= 16; i++) {
      aprilTagIdChooser.addOption(String.valueOf(i), i);
    }
    aprilTagIdChooser.setDefaultOption("1", 1);
    try {
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    }
    catch (IOException e) {
      DriverStation.reportError("Couldn't load apriltag field layout", true);
    }
  }

  @Override
  public void updatePoseEstimate(SwerveDrivePoseEstimator estimator) {
    if (hasTargets()) {
      var bestTarget = getBestTarget();
      var targetPoseOptional = aprilTagLayout.getTagPose(bestTarget.getFiducialId());
      if (targetPoseOptional.isEmpty()) {
        return;
      }
      var targetPose = targetPoseOptional.get();
      Transform3d cameraToRobot = getCameraToRobotTransform();
      Transform3d targetToCamera = new Transform3d(
          new Translation3d(
              getDistanceToBestTarget(),
              new Rotation3d(0, 0, Math.toRadians(-getAngleToBestTarget()))),
          cameraToRobot.getRotation()).inverse();
      Pose3d cameraPose = targetPose.transformBy(targetToCamera);
      Pose3d robotPose = cameraPose.transformBy(cameraToRobot);

      estimator.addVisionMeasurement(robotPose.toPose2d(), getTargetTimestamp());
      targetXLogger.append(targetPose.getX());
      targetYLogger.append(targetPose.getY());
      targetAngleLogger.append(Math.toRadians(targetPose.getRotation().getAngle()));
    }
  }

  /**
   * Returns the AprilTag target of the input ID.
   * 
   * @param id The AprilTag ID.
   * @return The target with the input ID.
   */
  public Optional<PhotonTrackedTarget> getTarget(int id) {
    return getTargets().stream().filter(target -> target.getFiducialId() == id).findFirst();
  }

  /**
   * Returns an optional of the transform from the camera to the AprilTag with input ID. 
   * @param id The AprilTag ID.
   * @return The transform from the camera to the AprilTag with input ID.
   */
  public Optional<Transform3d> getCameraToTarget(int id) {
    if (getTarget(id).isEmpty()) {
      return Optional.empty();
    }
      
    return Optional.of(getTarget(id).get().getBestCameraToTarget());
  }

  /**
   * Returns the distance to the target with the input ID. Returns 0 if target not
   * found.
   * 
   * @param id The AprilTag ID.
   * @return The distance to the target with the input ID.
   */
  public double getDistanceToTarget(int id) {
    Optional<PhotonTrackedTarget> target = getTarget(id);
    if (target.isEmpty()) {
      return 0.0;
    }
    var bestCameraToTarget = target.get().getBestCameraToTarget();
    return Math.hypot(bestCameraToTarget.getX(), bestCameraToTarget.getY());
  }

  /**
   * Returns the angle to the target with the input ID. Returns 0 if target not
   * found.
   * 
   * @param id The AprilTag ID.
   * @return The angle to the target with the input ID.
   */
  public double getAngleToTarget(int id) {
    Optional<PhotonTrackedTarget> target = getTarget(id);
    if (target.isEmpty()) {
      return 0.0;
    }
    return target.get().getYaw();
  }

  public static int getSpeakerCenterAprilTagID() {
    var alliance = DriverStation.getAlliance().get();
    return alliance == Alliance.Red ? 4 : 7;
  }

  public static int getAmpAprilTagID() {
    var alliance = DriverStation.getAlliance().get();
    return alliance == Alliance.Red ? 5 : 6;
  }

  /**
   * Returns an optional Pose3d that represents the field position of the tag (origin at blue).
   * 
   * @param id The ID of the target tag.
   * @return A Pose3d that represents the field position of the tag.
   */
  public Optional<Pose3d> getTagPose(int id) {
    return tagLayout.getTagPose(id);
  }

  /**
   * Returns the estimated field relative Pose3d of the robot (origin at blue). Returns a Pose3d with x, y, z = -1.0 if tag not found.
   * 
   * @param id The ID of the target tag.
   * @return the estimated field relative Pose3d of the robot.
   */
  public Pose3d getEstimatedRobotPose(int id) {
    if (getCameraToTarget(id).isEmpty()) {
      return new Pose3d(-1.0, -1.0, -1.0, new Rotation3d());
    }
    return getTagPose(id).get().transformBy(getCameraToTarget(id).get().inverse()).transformBy(getCameraToRobotTransform());
  }

  /**
   * Adds a tab for April Tag in Shuffleboard.
   */
  public void addShuffleboardTab() {
    if (!enableTab.getValue()) {
      return;
    }

    ShuffleboardTab visionTab = Shuffleboard.getTab("April Tag");

    ShuffleboardLayout targetLayout = visionTab.getLayout("Target Info", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(2, 5);
    targetLayout.add("Id Selection", aprilTagIdChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    targetLayout.addBoolean("Has Target", this::hasTargets);
    targetLayout.addDouble("Distance", () -> getDistanceToTarget(aprilTagIdChooser.getSelected()));
    targetLayout.addDouble("Angle", () -> getAngleToTarget(aprilTagIdChooser.getSelected()));

    ShuffleboardLayout estimationLayout = visionTab.getLayout("Pose Estimation Info", BuiltInLayouts.kList)
      .withPosition(6, 0)
      .withSize(2, 4);
    estimationLayout.addDouble("Tag pose X", () -> getTagPose(aprilTagIdChooser.getSelected()).get().getX());
    estimationLayout.addDouble("Tag pose Y", () -> getTagPose(aprilTagIdChooser.getSelected()).get().getY());
    estimationLayout.addDouble("Estimated Robot X", () -> getEstimatedRobotPose(aprilTagIdChooser.getSelected()).getX());
    estimationLayout.addDouble("Estimated Robot Y", () -> getEstimatedRobotPose(aprilTagIdChooser.getSelected()).getY());
    estimationLayout.addDouble("Estimated Robot Z", () -> getEstimatedRobotPose(aprilTagIdChooser.getSelected()).getZ());
  
    VideoSource video = new HttpCamera("photonvision_Port_1182_Output_MJPEG_Server", "http://photonvision.local:1182/?action=stream",
        HttpCameraKind.kMJPGStreamer);
    visionTab.add("April Tag", video)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);
  }
}