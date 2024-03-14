/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.RobotConstants;

@RobotPreferencesLayout(
    groupName = "NoteVision",
    column = 1,
    row = 2,
    width = 2,
    height = 2,
    type = "Grid Layout",
    gridColumns = 3,
    gridRows = 2)
public class NoteVisionSubsystem extends PhotonVisionSubsystemBase {
  @RobotPreferencesValue(column = 0, row = 0)
  public static final RobotPreferences.BooleanValue ENABLED =
      new RobotPreferences.BooleanValue("NoteVision", "Enabled", true);

  @RobotPreferencesValue(column = 1, row = 0)
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("NoteVision", "Enable Tab", false);

  // TODO: determine real pipeline
  public static final int NOTE_PIPELINE_INDEX = 0;
  public static final int APRILTAG_PIPELINE_INDEX = 1;

  /** Creates a new NoteSubsystem. */
  public NoteVisionSubsystem() {
    super("948ColorCamera", RobotConstants.NOTE_ROBOT_TO_CAMERA);
    camera.setPipelineIndex(NOTE_PIPELINE_INDEX);
  }

  /** Set the camera pipeline to Apriltag detection. */
  public void setApriltagPipeline() {
    camera.setPipelineIndex(APRILTAG_PIPELINE_INDEX);
  }

  /** Set the camera pipeline to note detection. */
  public void setNotePipeline() {
    camera.setPipelineIndex(NOTE_PIPELINE_INDEX);
  }

  /**
   * Returns the index of current pipeline on the camera.
   *
   * @return The index of current pipeline on the camera.
   */
  public int getCurrentPipelineIndex() {
    return camera.getPipelineIndex();
  }

  /** Adds a tab for Note Vision in Shuffleboard. */
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }

    ShuffleboardTab noteVisionTab = Shuffleboard.getTab("Note Detection");
    ShuffleboardLayout targetLayout =
        noteVisionTab
            .getLayout("Target Info", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 5);
    targetLayout.addBoolean("Has Target", this::hasTargets);
    targetLayout.addDouble("Angle", this::getAngleToBestTarget);
    targetLayout.addDouble("Pipeline Index", this::getCurrentPipelineIndex);

    VideoSource video =
        new HttpCamera(
            "photonvision_Port_1182_Output_MJPEG_Server",
            "http://photonvision.local:1182/?action=stream",
            HttpCameraKind.kMJPGStreamer);
    noteVisionTab
        .add("Note Camera", video)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);
  }
}
