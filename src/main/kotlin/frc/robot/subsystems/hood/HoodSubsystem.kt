package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.sensors.AbsoluteSensorRange
import com.ctre.phoenix.sensors.WPI_CANCoder
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.compareTo
import com.hamosad1657.lib.units.degrees
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

	private val motor = HaTalonFX(RobotMap.Hood.MOTOR_ID).apply {
		inverted = false // TODO: verify positive output makes the angle higher (more positive)
		configRemoteFeedbackFilter(encoder, 0)
		configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
		configForwardSoftLimitThreshold((Constants.MAX_HOOD_ANGLE * GEAR_RATIO_ENCODER_TO_HOOD).degrees)
		configForwardSoftLimitThreshold((Constants.MIN_HOOD_ANGLE * GEAR_RATIO_ENCODER_TO_HOOD).degrees)
		configForwardSoftLimitEnable(true)
		configReverseSoftLimitEnable(true)
		configPIDGains(Constants.PID_GAINS)
	}

	private val encoder = WPI_CANCoder(RobotMap.Hood.ENCODER_ID).apply {
		configSensorDirection(false) // TODO: verify more positive measurement is higher hood angle
		configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
	}

	/** Pressed at lowest hood angle */
	private val bottomLimitSwitch = DigitalInput(RobotMap.Hood.BOTTOM_LIMIT_CHANNEL)

	/** Pressed at highest hood angle */
	private val topLimitSwitch = DigitalInput(RobotMap.Hood.TOP_LIMIT_CHANNEL)

	private val currentAngle: Rotation2d get() = Rotation2d.fromDegrees(encoder.position * GEAR_RATIO_ENCODER_TO_HOOD)
	private var setpoint = Rotation2d()
	private val error
		get() = setpoint - currentAngle

	init {
		motor.forwardLimit = { currentAngle >= Constants.MAX_HOOD_ANGLE }
		motor.reverseLimit = { bottomLimitSwitch.get() }
		SmartDashboard.putData(this)
	}

	fun getToAngle(desiredAngle: Rotation2d) {
		val clampedDesiredAngleDeg =
			clamp(desiredAngle.degrees, Constants.MIN_HOOD_ANGLE.degrees, Constants.MAX_HOOD_ANGLE.degrees)

		this.setpoint = clampedDesiredAngleDeg.degrees

		val setpointDeg = clampedDesiredAngleDeg / GEAR_RATIO_ENCODER_TO_HOOD
		motor.set(ControlMode.Position, setpointDeg)
	}

	fun stopHood() {
		motor.stopMotor()
	}

	fun withinTolerance(): Boolean {
		val error = setpoint - currentAngle
		return abs(error.degrees) <= HoodConstants.ANGLE_TOLERANCE.degrees
	}

	private fun setWithLimits(controlMode: ControlMode, value: Double) {
		if (error.degrees > 0.0 && !topLimitSwitch.get() ||
			error.degrees < 0.0 && !bottomLimitSwitch.get()
		) { // Switches are wired normally true
			motor.stopMotor()
		} else {
			motor.set(controlMode, value)
		}
	}

	override fun initSendable(builder: SendableBuilder) {
		builder.setSmartDashboardType("HoodSubsystem")
		builder.addDoubleProperty("Current Angle deg", { currentAngle.degrees }, null)
		builder.addDoubleProperty("Encoder angle deg", { encoder.position }, null)
		builder.addBooleanProperty("Within tolerance", ::withinTolerance, null)
	}
}
