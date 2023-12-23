package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.*
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.WPI_CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.hood.HoodConstants
import frc.robot.subsystems.hood.HoodConstants.GEAR_RATIO_ENCODER_TO_HOOD
import kotlin.math.abs
import frc.robot.subsystems.hood.HoodConstants as Constants


object HoodSubsystem : SubsystemBase() {
	private val encoder = WPI_CANCoder(RobotMap.Hood.ENCODER_ID).apply {
		configSensorDirection(false) // Positive angle is higher.
		configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
	}

	private val motor = HaTalonFX(RobotMap.Hood.MOTOR_ID).apply {
		inverted = false // Positive output is higher angle
		configRemoteFeedbackFilter(encoder, 0)
		configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
		configForwardSoftLimitThreshold(Constants.MAX_ANGLE.degrees * GEAR_RATIO_ENCODER_TO_HOOD)
		configForwardSoftLimitThreshold(Constants.MIN_ANGLE.degrees * GEAR_RATIO_ENCODER_TO_HOOD)
		configForwardSoftLimitEnable(true)
		configReverseSoftLimitEnable(true)
		configPIDGains(Constants.PID_GAINS)
		setNeutralMode(NeutralMode.Brake)
	}

	/** Pressed at lowest hood angle */
	private val bottomLimitSwitch = DigitalInput(RobotMap.Hood.BOTTOM_LIMIT_CHANNEL)

	// Switch is wired normally true, false when pressed
	val isAtBottomLimit get() = !bottomLimitSwitch.get() || currentAngle < Constants.MIN_ANGLE
	val isAtTopLimit get() = currentAngle > Constants.MAX_ANGLE

	var currentAngle = Rotation2d()
		get() = Rotation2d.fromDegrees(encoder.position / GEAR_RATIO_ENCODER_TO_HOOD)
		private set

	private var setpoint = Rotation2d()
	private val error
		get() = setpoint - currentAngle

	init {
		SmartDashboard.putData(this)
	}

	fun getToAngle(desiredAngle: Rotation2d) {
		val clampedDesiredAngleDeg =
			clamp(desiredAngle, Constants.MIN_ANGLE, Constants.MAX_ANGLE).degrees

		this.setpoint = clampedDesiredAngleDeg.degrees

		val setpointDeg = clampedDesiredAngleDeg * GEAR_RATIO_ENCODER_TO_HOOD
		setWithLimits(ControlMode.Position, setpointDeg)
	}

	fun stopHood() {
		setpoint = currentAngle
		motor.stopMotor()
	}

	fun withinTolerance(): Boolean {
		val error = setpoint - currentAngle
		return abs(error.degrees) <= HoodConstants.ANGLE_TOLERANCE.degrees
	}

	private fun setWithLimits(controlMode: ControlMode, value: Double) {
		if (error.degrees > 0.0 && isAtTopLimit ||
			error.degrees < 0.0 && isAtBottomLimit
		) { // Switches are wired normally true
			stopHood()
		} else {
			motor.set(controlMode, value)
		}
	}

	override fun periodic() {
		// Switch is wired normally true, false when pressed
		if (!bottomLimitSwitch.get()) {
			currentAngle = Constants.MIN_ANGLE
		}
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("HoodSubsystem")
		builder.addBooleanProperty("Bottom angle limit", { isAtBottomLimit }, null)
		builder.addBooleanProperty("Top angle angle limit", { isAtTopLimit }, null)
		builder.addDoubleProperty("Current Angle deg", { currentAngle.degrees }, null)
		builder.addDoubleProperty("Encoder angle deg", { encoder.position }, null)
		builder.addBooleanProperty("Within tolerance", ::withinTolerance, null)
	}
}
