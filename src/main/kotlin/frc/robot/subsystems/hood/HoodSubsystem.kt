package frc.robot.subsystems.hood

import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaCANSparkMax
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object HoodSubsystem: SubsystemBase() {
	private val motor = HaCANSparkMax(RobotMap.Hood.MOTOR_ID)
	private val encoder = CANCoder(RobotMap.Hood.ENCODER_ID)


	private val reverseLimit = DigitalInput(RobotMap.Hood.LIMIT_CHANNEL)

	init {
		motor.forwardLimit = {getHoodAngle().degrees >= HoodConstants.maxHoodAngle}

		motor.reverseLimit = {reverseLimit.get()}

	}

	fun getHoodAngle(): Rotation2d {
		return Rotation2d.fromDegrees(encoder.position * HoodConstants.gearRatioEncoderToHood)
	}
}