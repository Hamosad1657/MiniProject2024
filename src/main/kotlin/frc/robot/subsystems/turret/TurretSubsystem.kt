package frc.robot.subsystems.turret

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.WPI_CANCoder
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.math.wrap0to360
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.compareTo
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.turret.TurretConstants
import org.photonvision.targeting.PhotonTrackedTarget
import kotlin.math.abs
import frc.robot.subsystems.turret.TurretConstants as Constants

object TurretSubsystem : SubsystemBase() {
	val motor = HaTalonFX(RobotMap.Turret.MOTOR_ID).apply {
		inverted = false // TODO: verify Turret motor is CCW positive
		forwardLimit = { currentAngle >= Constants.MAX_ANGLE }
		reverseLimit = { currentAngle <= Constants.MIN_ANGLE }

		configPIDGains(Constants.PID_GAINS)
	}

	private val encoder = WPI_CANCoder(RobotMap.Turret.CANCODER_ID).apply {
		configSensorDirection(false) // TODO: verify Turret CANCoder is CCW positive
		configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
	}

	/** CCW positive, according to standard mathematical conventions (and WPILib). */
	val currentAngle: Rotation2d get() = Rotation2d.fromDegrees(encoder.position * Constants.GEAR_RATIO_ENCODER_TO_TURRET)
	val farthestTurnAngle get() = if (currentAngle.degrees >= 180) Constants.MIN_ANGLE else Constants.MAX_ANGLE

	private var setpoint = Rotation2d()

	private val tagDetectionDebouncer = Debouncer(Constants.TAG_DETECTION_TIME_SEC)

	init {
		SmartDashboard.putData(this)
		getToAngle(Constants.MIN_ANGLE)
	}

	/**
	 * @param desiredAngle Can be any value, is not required to be in 0 to 360
	 */
	fun getToAngle(desiredAngle: Rotation2d) {
		val wrappedDesiredAngle = wrap0to360(desiredAngle.degrees)
		val clampedDesiredAngle = clamp(wrappedDesiredAngle, Constants.MIN_ANGLE.degrees, Constants.MAX_ANGLE.degrees)
		val setpointDeg = clampedDesiredAngle / Constants.GEAR_RATIO_ENCODER_TO_TURRET
		motor.set(ControlMode.Position, setpointDeg)
	}

	fun guessTurnAngleFromTargetCorner(cornerX: Double) =
		if (cornerX > Constants.CAMERA_WIDTH / 2.0) Constants.MAX_ANGLE else Constants.MIN_ANGLE

	/**
	 * Returns is a tag of the specified [tagID] was detected for the last [Constants.TAG_DETECTION_TIME_SEC] seconds.
	 */
	fun isTagDetected(tagID: Int, trackedTarget: PhotonTrackedTarget?): Boolean {
		return tagDetectionDebouncer.calculate(trackedTarget != null && trackedTarget.fiducialId == tagID)
	}

	/**
	 * Returns if a tag was detected for the last [Constants.TAG_DETECTION_TIME_SEC] seconds.
	 */
	fun isAnyTagDetected(trackedTarget: PhotonTrackedTarget?): Boolean {
		return tagDetectionDebouncer.calculate(trackedTarget != null)
	}

	fun withinTolerance(): Boolean {
		val error = setpoint - currentAngle
		return error.degrees <= abs(TurretConstants.TOLERANCE_DEGREES)
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("TurretSubsystem")
		builder.addDoubleProperty("Robot-relative angle deg", { currentAngle.degrees }, null)
		builder.addDoubleProperty("Encoder degrees", { encoder.position }, null)
		builder.addBooleanProperty("Within Tolerance", ::withinTolerance, null)
	}
}
