package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import com.hamosad1657.lib.units.degrees
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.hood.HoodConstants.GEAR_RATIO_ENCODER_TO_HOOD


object HoodSubsystem : SubsystemBase() {
	private val encoder = CANCoder(RobotMap.Hood.ENCODER_ID)

	val motor = HaTalonFX(RobotMap.Hood.MOTOR_ID).apply {
		configRemoteFeedbackFilter(frc.robot.subsystems.hood.HoodSubsystem.encoder, 0)
		configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
		config_kP(0, HoodConstants.PID_GAINS.kP)
		config_kP(0, HoodConstants.PID_GAINS.kI)
		config_kP(0, HoodConstants.PID_GAINS.kD)
	}

	private val reverseLimitSwitch = DigitalInput(RobotMap.Hood.LIMIT_CHANNEL)

	var setpoint = 0.degrees
		private set(value) {
			motor.set(ControlMode.Position, value.degrees)
			field = setpoint
		}

	val angle: Rotation2d
		get() = Rotation2d.fromDegrees(encoder.position * GEAR_RATIO_ENCODER_TO_HOOD)

	init {
		motor.forwardLimit = { angle.degrees >= HoodConstants.MAX_HOOD_ANGLE_DEG }
		motor.reverseLimit = { reverseLimitSwitch.get() }
	}

	fun getToAngle(desiredAngle: Rotation2d) {
		setpoint = desiredAngle
	}

}