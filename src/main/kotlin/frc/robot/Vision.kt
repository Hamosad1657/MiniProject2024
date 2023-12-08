package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import kotlin.math.PI

object Vision {
	val aprilTags: AprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField()

	private val robotToCamera = Transform3d(Translation3d(), Rotation3d(0.0, 0.0, -PI / 2))
	private val camera = PhotonCamera("AprilTag-Cam")
	private val poseEstimator =
		PhotonPoseEstimator(
			aprilTags,
			PoseStrategy.MULTI_TAG_PNP,
			camera,
			robotToCamera,
		).apply {
			setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
		}

	/**
	 * Gets the estimated robot position from the PhotonVision camera.
	 * Returns null if it doesn't detect any April Tags.
	 */
	val estimatedGlobalPose: EstimatedRobotPose? get() = poseEstimator.update().orElse(null)
}