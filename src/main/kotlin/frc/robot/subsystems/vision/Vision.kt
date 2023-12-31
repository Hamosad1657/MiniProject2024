package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.photonvision.*
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import kotlin.math.PI

object Vision : SubsystemBase() {

	val aprilTags: AprilTagFieldLayout =
		AprilTagFieldLayout(
			mutableListOf(
				AprilTag(
					8,
					Pose3d(
						0.0, 0.0, 0.45,
						Rotation3d(0.0, 0.0, 0.0)
					)
				),
				AprilTag(
					4,
					Pose3d(
						0.0, 0.35, 0.45,
						Rotation3d(0.0, 0.0, 0.0)
					)
				)
			), 10.0, 10.0

		)


	private val robotToCamera = Transform3d(Translation3d(-0.4, -0.2, 0.56), Rotation3d(-PI / 2, 0.0, -PI / 2))

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
	var estimatedGlobalPose: EstimatedRobotPose? = poseEstimator.update().orElse(null)


	override fun periodic() {
		estimatedGlobalPose = poseEstimator.update().orElse(null)
	}
}