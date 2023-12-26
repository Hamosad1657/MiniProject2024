package frc.robot.subsystems.turret

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.sensors.WPI_CANCoder
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.math.wrap0to360
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.turret.TurretConstants
import kotlin.math.abs
import frc.robot.subsystems.turret.TurretConstants as Constants

object TurretSubsystem : SubsystemBase(), AutoCloseable {
	private val encoder = WPI_CANCoder(RobotMap.Turret.CANCODER_ID).apply {
		configSensorDirection(true) // Turret CANCoder is CCW positive
	}

	private val motor = HaTalonFX(RobotMap.Turret.MOTOR_ID).apply {
		inverted = false // Positive output is CCW rotation
		configForwardSoftLimitThreshold(Constants.MIN_POSITION)
		configReverseSoftLimitThreshold(Constants.MAX_POSITION)
		configForwardSoftLimitEnable(false)
		configReverseSoftLimitEnable(false)

		configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed)
		configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed)

		configPIDGains(Constants.PID_GAINS)

		// setNeutralMode(NeutralMode.Brake)
		setNeutralMode(NeutralMode.Coast) // TODO: Change turret back to brake when done testing ratios and limits and such
	}

	/** CCW positive, according to standard mathematical conventions (and WPILib). */
	var currentAngle = Rotation2d()
		get() = ((encoder.position / Constants.GEAR_RATIO_ENCODER_TO_TURRET) - 90).degrees
		private set

	private val CWLimitSwitch = DigitalInput(RobotMap.Turret.CW_LIMIT_CHANNEL)
	private val CCWLimitSwitch = DigitalInput(RobotMap.Turret.CCW_LIMIT_CHANNEL)

	// Switches are wired normally false, true when pressed
	val isAtCWLimit get() = CWLimitSwitch.get()
	val isAtCCWLimit get() = CCWLimitSwitch.get()

	val farthestTurnAngle get() = if (currentAngle.degrees >= 180) Constants.MIN_ANGLE else Constants.MAX_ANGLE

	private var setpoint = Rotation2d()

	private val error get() = setpoint - currentAngle

	init {
		SmartDashboard.putData(this)
	}

	fun stopTurret() {
//		setSetpoint(currentAngle.degrees)
		motor.stopMotor()
	}

	/**
	 * @param desiredAngle Can be any value, is not required to be in 0 to 360
	 */
	fun getToAngle(desiredAngle: Rotation2d) {
		val wrappedDesiredAngleDeg = wrap0to360(desiredAngle.degrees)
		val clampedDesiredAngleDeg =
			clamp(wrappedDesiredAngleDeg, Constants.MIN_ANGLE.degrees, Constants.MAX_ANGLE.degrees)

		val setpointDeg = clampedDesiredAngleDeg * Constants.GEAR_RATIO_ENCODER_TO_TURRET
		setWithLimits(ControlMode.Position, setpointDeg)
	}

	fun guessTurnAngleFromTargetCorner(cornerX: Double) =
		if (cornerX > Constants.CAMERA_WIDTH / 2.0) Constants.MAX_ANGLE else Constants.MIN_ANGLE

	fun withinTolerance(): Boolean {
		return abs(error.degrees) <= TurretConstants.TOLERANCE_DEGREES
	}

	fun setWithLimits(controlMode: ControlMode, value: Double) {
		when (controlMode) {
			ControlMode.PercentOutput -> {
				this.setpoint = value.degrees

				if (value > 0.0 && isAtCCWLimit ||
					value < 0.0 && isAtCWLimit
				) {
					stopTurret()
				} else {
					motor.set(value)
				}
			}

			ControlMode.Position -> {
				if (error.degrees > 0.0 && isAtCCWLimit ||
					error.degrees < 0.0 && isAtCWLimit
				) {
					stopTurret()
				} else {
					motor.set(ControlMode.Position, value)
				}
			}

			else -> {
				throw IllegalArgumentException()
			}
		}
	}

	override fun periodic() {
		// Switch is wired normally false, true when pressed
		if (isAtCWLimit) {
			currentAngle = Constants.MIN_ANGLE
		}
		if (isAtCCWLimit) {
			currentAngle = Constants.MAX_ANGLE
		}
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("TurretSubsystem")
		builder.addDoubleProperty("Error", { error.degrees }, null)
		builder.addDoubleProperty("Robot-relative angle deg", { currentAngle.degrees }, null)
		builder.addDoubleProperty("Encoder degrees", { encoder.position }, null)
		builder.addBooleanProperty("Within Tolerance", ::withinTolerance, null)
		builder.addBooleanProperty("CW limit", { isAtCWLimit }, null)
		builder.addBooleanProperty("CCW limit", { isAtCCWLimit }, null)
	}

	override fun close() {
		motor.close()
		encoder.close()
		CWLimitSwitch.close()
		CCWLimitSwitch.close()
	}

	// TODO: Remove resetAngle() when done testing with it
	fun zeroEncoderAngle() {
		encoder.position = 0.0
	}
}
