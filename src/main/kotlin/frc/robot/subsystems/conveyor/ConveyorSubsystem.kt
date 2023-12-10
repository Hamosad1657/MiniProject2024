package frc.robot.subsystems.conveyor

import com.hamosad1657.lib.motors.HaCANSparkMax
import com.revrobotics.CANSparkMax.ControlType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.conveyor.ConveyorConstants as Constants

object ConveyorSubsystem : SubsystemBase() {
	private val loaderMotor = HaCANSparkMax(RobotMap.Conveyor.LOADER_MOTOR_ID)
	private val loaderEncoder = loaderMotor.encoder
	private val loaderPIDController = loaderMotor.pidController

	private val conveyorMotor = HaCANSparkMax(RobotMap.Conveyor.CONVEYOR_MOTOR_ID)
	private val conveyorEncoder = conveyorMotor.encoder
	private val conveyorPIDController = conveyorMotor.pidController

	val conveyorBallsPerSecs: Double
		get() = conveyorEncoder.velocity * Constants.CONVEYOR_VELOCITY_RATIO

	val loaderBallsPerSec: Double
		get() = loaderEncoder.velocity * Constants.LOADER_VELOCITY_RATIO

	fun runConveyor(ballsPerSecs: Double) {
		val setpointRPM = ballsPerSecs * Constants.CONVEYOR_VELOCITY_RATIO
		conveyorPIDController.setReference(setpointRPM, ControlType.kVelocity)
	}

	fun runLoader(ballsPerSecs: Double) {
		val setpointRPM = ballsPerSecs * Constants.LOADER_VELOCITY_RATIO
		loaderPIDController.setReference(setpointRPM, ControlType.kVelocity)
	}

	fun stopConveyor() {
		conveyorMotor.stopMotor()
	}

	fun stopLoader() {
		loaderMotor.stopMotor()
	}
}