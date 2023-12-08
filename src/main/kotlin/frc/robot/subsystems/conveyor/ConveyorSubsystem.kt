package frc.robot.subsystems.conveyor

import com.ctre.phoenix.motorcontrol.ControlMode
import com.hamosad1657.lib.motors.HaTalonFX
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotMap
import frc.robot.subsystems.conveyor.ConveyorConstants as Constants

object ConveyorSubsystem : SubsystemBase() {
	private val loaderMotor = HaTalonFX(RobotMap.Conveyor.LOADER_MOTOR_ID).apply {
		configPIDGains(Constants.LOADER_PID_GAINS)
	}
	private val conveyorMotor = HaTalonFX(RobotMap.Conveyor.CONVEYOR_MOTOR_ID).apply {
		configPIDGains(Constants.CONVEYOR_PID_GAINS)
	}

	val conveyorBallsPerSecs: Double
		get() = conveyorMotor.selectedSensorVelocity * Constants.CONVEYOR_VELOCITY_RATIO

	val loaderBallsPerSec: Double
		get() = loaderMotor.selectedSensorVelocity * Constants.LOADER_VELOCITY_RATIO

	fun runConveyor(ballsPerSecs: Double) {
		conveyorMotor.set(ControlMode.Velocity, ballsPerSecs)
	}

	fun runLoader(ballsPerSecs: Double) {
		val setpoint = Constants.CONVEYOR_VELOCITY_RATIO * ballsPerSecs
		loaderMotor.set(ControlMode.Velocity, setpoint)
	}

	fun stopConveyor() {
		conveyorMotor.stopMotor()
	}

	fun stopLoader() {
		loaderMotor.stopMotor()
	}
}