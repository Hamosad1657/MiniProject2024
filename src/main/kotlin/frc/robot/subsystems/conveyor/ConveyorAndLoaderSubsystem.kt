package frc.robot.subsystems.conveyor

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap

object ConveyorAndLoaderSubsystem : SubsystemBase() {
	private val loaderMotor = HaTalonFX(RobotMap.Loader.MOTOR_ID).apply {
		config_kP(0, frc.robot.subsystems.conveyor.ConveyorConstants.PID_GAINS.kP)
		config_kI(0, LoaderConstants.PID_GAINS.kI)
		config_kD(0, LoaderConstants.PID_GAINS.kD)
	}
	private val conveyorMotor = HaTalonFX(RobotMap.Conveyor.MOTOR_ID).apply {
		config_kP(0, frc.robot.subsystems.conveyor.ConveyorConstants.PID_GAINS.kP)
		config_kI(0, ConveyorConstants.PID_GAINS.kI)
		config_kD(0, ConveyorConstants.PID_GAINS.kD)
	}

	fun runConveyor(ballsPerSecs: Double) {
		conveyorMotor.set(ControlMode.Velocity, ballsPerSecs)
	}

	val conveyorBallsPerSecs: Double
		get() = conveyorMotor.selectedSensorVelocity * ConveyorConstants.CONVEYOR_VELOCITY_RATIO

	val loaderBallsPerSec: Double
		get() = loaderMotor.selectedSensorVelocity * LoaderConstants.LOADER_VELOCITY_RATIO

	fun runLoader(ballsPerSecs: Double) {
		val setpoint = ConveyorConstants.CONVEYOR_VELOCITY_RATIO * ballsPerSecs
		loaderMotor.set(ControlMode.Velocity, setpoint)
	}

	fun stopConveyor() {
		conveyorMotor.stopMotor()
	}

	fun stopLoader() {
		loaderMotor.stopMotor()
	}
}