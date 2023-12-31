package frc.robot.subsystems.swerve

import com.pathplanner.lib.*
import com.pathplanner.lib.auto.SwerveAutoBuilder
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.vision.Vision
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.parser.SwerveDriveConfiguration
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
import java.io.File
import kotlin.math.pow
import frc.robot.subsystems.swerve.SwerveConstants as Constants

object SwerveSubsystem : SubsystemBase() {
	private val swerveDrive: SwerveDrive

	/** Gets the current field-relative velocity (x, y and omega) of the robot*/
	val fieldVelocity: ChassisSpeeds get() = swerveDrive.fieldVelocity

	/** Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped. */
	val heading: Rotation2d get() = swerveDrive.yaw

	/** Get the swerve drive kinematics object.*/
	val kinematics: SwerveDriveKinematics get() = swerveDrive.kinematics

	/** Gets the current pitch angle of the robot, as reported by the imu. */
	val pitch: Rotation2d get() = swerveDrive.pitch

	/** Gets the current pose (position and rotation) of the robot, as reported by odometry. */
	val pose: Pose2d get() = swerveDrive.pose

	/** Gets the current velocity (x, y and omega) of the robot */
	val robotVelocity: ChassisSpeeds get() = swerveDrive.robotVelocity

	/** Get the [SwerveController] in the swerve drive. */
	val swerveController: SwerveController get() = swerveDrive.swerveController

	/** Get the [SwerveDriveConfiguration] object. */
	val swerveDriveConfiguration: SwerveDriveConfiguration get() = swerveDrive.swerveDriveConfiguration

	init {
		// Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW

		try {
			val swerveDirectory = File(Filesystem.getDeployDirectory(), Constants.SWERVE_CONFIG_DIR)
			swerveDrive = SwerveParser(swerveDirectory).createSwerveDrive()
		} catch (e: Exception) {
			throw RuntimeException(e)
		}

		setMotorBrake(true)

		swerveDrive.enableSecondOrderKinematics()
		SmartDashboard.putData(swerveDrive.field)
	}

	/**
	 * The auto builder for PathPlanner, there can only ever be one created,
	 * so we save it just in case we generate multiple paths with events.
	 */
	private var autoBuilder = SwerveAutoBuilder(
		{ swerveDrive.pose },
		{ pose: Pose2d? -> swerveDrive.resetOdometry(pose) },
		Constants.PATH_TRANSLATION_CONSTANTS,
		Constants.PATH_ROTATION_CONSTANTS,
		{ chassisSpeeds: ChassisSpeeds? ->
			swerveDrive.setChassisSpeeds(
				chassisSpeeds
			)
		},
		emptyMap(),
		false,
		this
	)

	/**
	 * The primary method for controlling the drivebase.  Takes a [Translation2d] and a rotation rate, and
	 * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
	 * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.
	 *
	 * @param translation   [Translation2d] that is the commanded linear velocity of the robot, in meters per
	 * second. In robot-relative mode, positive x is towards the bow (front) and positive y is
	 * towards port (left).  In field-relative mode, positive x is away from the alliance wall
	 * (field North) and positive y is towards the left wall when looking through the driver station
	 * glass (field West).
	 * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
	 * relativity.
	 * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
	 * @param isOpenLoop    Whether to use closed-loop velocity control.  Set to true to disable closed-loop.
	 */
	fun drive(translation: Translation2d?, rotation: Double, fieldRelative: Boolean, isOpenLoop: Boolean) {
		swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop)
	}

	/**
	 * Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
	 * the angle of the robot.
	 *
	 * @param xInput   X joystick input for the robot to move in the X direction.
	 * @param yInput   Y joystick input for the robot to move in the Y direction.
	 * @param headingX X joystick which controls the angle of the robot.
	 * @param headingY Y joystick which controls the angle of the robot.
	 * @return [ChassisSpeeds] which can be sent to th Swerve Drive.
	 */
	fun getTargetSpeeds(xInput: Double, yInput: Double, headingX: Double, headingY: Double): ChassisSpeeds {
		return swerveDrive.swerveController.getTargetSpeeds(
			xInput.pow(3.0),
			yInput.pow(3.0),
			headingX,
			headingY,
			heading.radians,
		)
	}

	/**
	 * Get the chassis speeds based on controller input of 1 joystick and one angle.
	 *
	 * @param xInput X joystick input for the robot to move in the X direction.
	 * @param yInput Y joystick input for the robot to move in the Y direction.
	 * @param angle  The angle in as a [Rotation2d].
	 * @return [ChassisSpeeds] which can be sent to th Swerve Drive.
	 */
	fun getTargetSpeeds(xInput: Double, yInput: Double, angle: Rotation2d): ChassisSpeeds {
		return swerveDrive.swerveController.getTargetSpeeds(
			xInput.pow(3.0),
			yInput.pow(3.0),
			angle.radians,
			heading.radians,
		)
	}

	/** Lock the swerve drive to prevent it from moving. */
	fun lock() {
		swerveDrive.lockPose()
	}

	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory The trajectory to post.
	 */
	fun postTrajectory(trajectory: Trajectory?) {
		swerveDrive.postTrajectory(trajectory)
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
	 * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
	 * keep working.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	fun resetOdometry(initialHolonomicPose: Pose2d?) {
		swerveDrive.resetOdometry(initialHolonomicPose)
	}

	fun resetOdometry() {
		resetOdometry(Pose2d())
	}

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds Chassis Speeds to set.
	 */
	fun setChassisSpeeds(chassisSpeeds: ChassisSpeeds?) {
		swerveDrive.setChassisSpeeds(chassisSpeeds)
	}

	/** Sets the drive motors to brake/coast mode. */
	fun setMotorBrake(isBrake: Boolean) {
		swerveDrive.setMotorIdleMode(isBrake)
	}

	/** Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0. */
	fun zeroGyro() {
		swerveDrive.zeroGyro()

	}

	private fun addVisionMeasurement(
		translationStdDev: Double = 0.3, thetaStdDev: Double = 0.9,
	) {
		Vision.estimatedGlobalPose?.let { estimatedPose ->
			val soft = true

			swerveDrive.field.getObject("Vision_Robot").pose = estimatedPose.estimatedPose.toPose2d()
			swerveDrive.addVisionMeasurement(
				estimatedPose.estimatedPose.toPose2d(),
				estimatedPose.timestampSeconds,
				soft,
				Matrix(Nat.N3(), Nat.N1()).apply {
					this[0, 0] = translationStdDev
					this[1, 0] = translationStdDev
					this[2, 0] = thetaStdDev
				},
			)
			swerveDrive.setGyroOffset(swerveDrive.gyroRotation3d)
		}
	}

	fun createPathPlannerCommand(path: String, constraints: PathConstraints): Command {
		val pathGroup = PathPlanner.loadPathGroup(path, constraints)
		resetOdometry(pathGroup[0].initialHolonomicPose)
		return autoBuilder.fullAuto(pathGroup)
	}

	fun createDynamicPathPlannerCommand(
		pathName: String,
		constraints: PathConstraints = Constants.PATH_CONSTRAINTS,
	): Command {
		val pathGroup = PathPlanner.loadPathGroup(pathName, constraints)
		val currentToStartPath = generateDynamicPath(targetPose = pathGroup[0].initialHolonomicPose, constraints)
		pathGroup.add(index = 0, currentToStartPath)
		return autoBuilder.fullAuto(pathGroup)
	}

	fun generateDynamicPath(
		targetPose: Pose2d,
		constraints: PathConstraints = Constants.PATH_CONSTRAINTS,
	): PathPlannerTrajectory {
		val currentPose = swerveDrive.pose
		
		return PathPlanner.generatePath(
			constraints,
			PathPoint(
				currentPose.translation,
				(targetPose.translation - currentPose.translation).angle,
				currentPose.rotation
			),
			PathPoint(
				targetPose.translation,
				(targetPose.translation - currentPose.translation).angle,
				targetPose.rotation
			)
		)
	}

	fun followPathCommand(path: PathPlannerTrajectory): Command {
		postTrajectory(path)
		return autoBuilder.followPath(path)
	}

	override fun periodic() {
		addVisionMeasurement()
		SmartDashboard.putString("RobotPose", pose.toString())
	}
}
