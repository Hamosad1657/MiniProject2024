package frc.robot.subsystems.conveyor

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.sensors.CANCoder
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ConveyorSubsystem : SubsystemBase() {
	private val motor = HaTalonFX(RobotMap.Conveyor.MOTOR_ID)
	private val encoder = CANCoder(RobotMap.Conveyor.ENCODER_ID)

	init {
		motor.config_kP(0, ConveyorConstants.PID_GAINS.kP)
		motor.config_kI(0, ConveyorConstants.PID_GAINS.kI)
		motor.config_kD(0, ConveyorConstants.PID_GAINS.kD)

		motor.configRemoteFeedbackFilter(encoder, 0)
		motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0)
	}

	fun runConveyorCommand(ballsPerSecs: () -> Double): Command {
		return RunCommand({ runConveyor(ballsPerSecs()) }, this)

	}

	private fun runConveyor(ballsPerSecs: Double) {
		motor.set(ControlMode.Velocity, ballsPerSecs)
	}

	val ballsPerSecs: Double
		get() = encoder.velocity * ConveyorConstants.CONVEYOR_VELOCITY_CONVERSION
}