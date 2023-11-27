package frc.robot.subsystems.hood

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.hood.HoodConstants.GEAR_RATIO_ENCODER_TO_HOOD


object HoodSubsystem : SubsystemBase() {
	private val encoder = CANCoder(RobotMap.Hood.ENCODER_ID)
	private val motor = HaTalonFX(RobotMap.Hood.MOTOR_ID)
	private val reverseLimitSwitch = DigitalInput(RobotMap.Hood.LIMIT_CHANNEL)

	private val hoodAngleDeg: Double
		get() = encoder.position * GEAR_RATIO_ENCODER_TO_HOOD

	init {
		motor.forwardLimit = { hoodAngleDeg >= HoodConstants.MAX_HOOD_ANGLE }
		motor.reverseLimit = { reverseLimitSwitch.get() }

		motor.config_kP(0, HoodConstants.PID_GAINS.kP)
		motor.config_kP(0, HoodConstants.PID_GAINS.kI)
		motor.config_kP(0, HoodConstants.PID_GAINS.kD)
	}

	fun getToAngleCommand(desiredAngleSupplier: () -> Double): Command {
		return run {
			getToAngle(desiredAngleSupplier())
		}.finallyDo { motor.stopMotor() }
	}

	private fun getToAngle(desiredAngle: Double) {
		return motor.set(ControlMode.Position, desiredAngle)
	}

	val angle: Rotation2d
		get() = Rotation2d.fromDegrees(encoder.position * GEAR_RATIO_ENCODER_TO_HOOD)

}