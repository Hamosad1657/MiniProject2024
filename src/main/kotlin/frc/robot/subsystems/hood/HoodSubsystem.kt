package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.math.clamp
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.compareTo
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.hood.HoodConstants.GEAR_RATIO_ENCODER_TO_HOOD
import frc.robot.subsystems.hood.HoodConstants as Constants


object HoodSubsystem : SubsystemBase() {
	private val encoder = CANCoder(RobotMap.Hood.ENCODER_ID)
	private val reverseLimitSwitch = DigitalInput(RobotMap.Hood.LIMIT_CHANNEL)

	val motor = HaTalonFX(RobotMap.Hood.MOTOR_ID).apply {
		configRemoteFeedbackFilter(encoder, 0)
		configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
		configPIDGains(Constants.PID_GAINS)
	}

	private val currentAngle: Rotation2d get() = Rotation2d.fromDegrees(encoder.position * GEAR_RATIO_ENCODER_TO_HOOD)

	init {
		motor.forwardLimit = { currentAngle >= Constants.MAX_HOOD_ANGLE }
		motor.reverseLimit = { reverseLimitSwitch.get() }
	}

	fun getToAngle(desiredAngle: Rotation2d) {
		val clampedDesiredAngle =
			clamp(desiredAngle.degrees, Constants.MIN_HOOD_ANGLE.degrees, Constants.MAX_HOOD_ANGLE.degrees)
		val setpointDeg = clampedDesiredAngle / GEAR_RATIO_ENCODER_TO_HOOD
		motor.set(ControlMode.Position, setpointDeg)
	}
}