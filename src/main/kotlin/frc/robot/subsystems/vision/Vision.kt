package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.*
import org.photonvision.*
import org.photonvision.targeting.PhotonTrackedTarget

object Vision {
	val currentBestTag: PhotonTrackedTarget? = null

	val aprilTags: AprilTagFieldLayout =
		AprilTagFieldLayout(
			mutableListOf(
				AprilTag(
					0,
					Pose3d(
						0.0, 0.0, 0.0,
						Rotation3d(0.0, 0.0, 0.0)
					)
				),
				AprilTag(
					0,
					Pose3d(
						0.0, 0.0, 0.0,
						Rotation3d(0.0, 0.0, 0.0)
					)
				)
			), 0.0, 0.0
		)

	private val robotToCamera = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0))

	private val camera = PhotonCamera("")

	private val poseEstimator =
		PhotonPoseEstimator(
			aprilTags,
			PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP,
			camera,
			robotToCamera,
		).apply {
			setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
		}

	/**
	 * Gets the estimated robot position from the PhotonVision camera.
	 * Returns null if it doesn't detect any April Tags.
	 */
	var estimatedGlobalPose: EstimatedRobotPose? = poseEstimator.update().orElse(null)

}