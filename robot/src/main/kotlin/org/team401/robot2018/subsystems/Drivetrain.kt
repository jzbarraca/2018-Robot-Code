package org.team401.robot2018.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.snakeskin.component.Gearbox
import org.snakeskin.dsl.*
import org.snakeskin.component.TankDrivetrain
import org.snakeskin.event.Events
import org.team401.robot2018.Constants
import org.team401.robot2018.LeftStick
import org.team401.robot2018.RightStick
import java.lang.Thread.sleep

/*
 * 2018-Robot-Code - Created on 1/13/18
 * Author: Cameron Earle
 * 
 * This code is licensed under the GNU GPL v3
 * You can find more info in the LICENSE file at project root
 */

/**
 * @author Cameron Earle
 * @version 1/13/18
 */

const val DRIVE_MACHINE = "drive"
object DriveStates {
    const val OPEN_LOOP = "openloop"
}

val Drivetrain = TankDrivetrain(Constants.DrivetrainParameters.WHEEL_RADIUS, Constants.DrivetrainParameters.WHEELBASE)

val DrivetrainSubsystem: Subsystem = buildSubsystem {
    val leftFront = TalonSRX(Constants.MotorControllers.DRIVE_LEFT_FRONT_CAN)
    val leftMidF = TalonSRX(Constants.MotorControllers.DRIVE_LEFT_MIDF_CAN)
    val leftMidR = TalonSRX(Constants.MotorControllers.DRIVE_LEFT_MIDR_CAN)
    val leftRear = TalonSRX(Constants.MotorControllers.DRIVE_LEFT_REAR_CAN)
    val rightFront = TalonSRX(Constants.MotorControllers.DRIVE_RIGHT_FRONT_CAN)
    val rightMidF = TalonSRX(Constants.MotorControllers.DRIVE_RIGHT_MIDF_CAN)
    val rightMidR = TalonSRX(Constants.MotorControllers.DRIVE_RIGHT_MIDR_CAN)
    val rightRear = TalonSRX(Constants.MotorControllers.DRIVE_RIGHT_REAR_CAN)

    val left = Gearbox(leftFront, leftMidF, leftMidR, leftRear)
    val right = Gearbox(rightFront, rightMidF, rightMidR, rightRear)
    val imu = PigeonIMU(leftRear)

    val shifter = Solenoid(Constants.Pneumatics.SHIFTER_SOLENOID)

    setup {
        left.setSensor(FeedbackDevice.CTRE_MagEncoder_Absolute)
        right.setSensor(FeedbackDevice.CTRE_MagEncoder_Absolute)

        Drivetrain.init(left, right, imu, shifter, Constants.DrivetrainParameters.INVERT_LEFT, Constants.DrivetrainParameters.INVERT_RIGHT, Constants.DrivetrainParameters.INVERT_SHIFTER)
        Drivetrain.setCurrentLimit(Constants.DrivetrainParameters.CURRENT_LIMIT_CONTINUOUS, Constants.DrivetrainParameters.CURRENT_LIMIT_PEAK, Constants.DrivetrainParameters.CURRENT_LIMIT_TIMEOUT)
        Drivetrain.setRampRate(Constants.DrivetrainParameters.CLOSED_LOOP_RAMP, Constants.DrivetrainParameters.OPEN_LOOP_RAMP)

        println("----------SETUP DONE----------")
    }



    val driveMachine = stateMachine(DRIVE_MACHINE) {
        state(DriveStates.OPEN_LOOP) {
            entry {
                Drivetrain.zero()
            }
            action {
                Drivetrain.arcade(ControlMode.PercentOutput, LeftStick.readAxis { PITCH }, RightStick.readAxis { ROLL })
                println(left.master.getSelectedSensorVelocity(0))
            }
        }

        default {
            entry {
                Drivetrain.stop()
            }
        }
    }
    test("Drivetrain test"){
        println("Running drivetrain test")

        sleep(1000)
        leftFront.set(ControlMode.PercentOutput, 1.0)
        sleep(1000)
        //Not sure if pidIdx if correct
        val leftFrontEncPosition = leftFront.getSelectedSensorPosition(0)
        val leftFrontEncVelocity = leftFront.getSelectedSensorVelocity(0)
        SmartDashboard.putNumber("leftFront Position", leftFrontEncPosition.toDouble())
        SmartDashboard.putNumber("leftFront Velocity", leftFrontEncVelocity.toDouble())
        leftFront.set(ControlMode.PercentOutput, 0.0)

        sleep(1000)
        leftMidF.set(ControlMode.PercentOutput, 1.0)
        sleep(1000)
        //Not sure if pidIdx if correct
        val leftMidFEncPosition = leftMidF.getSelectedSensorPosition(0)
        val leftMidFEncVelocity = leftMidF.getSelectedSensorVelocity(0)
        SmartDashboard.putNumber("leftMidF Position", leftMidFEncPosition.toDouble())
        SmartDashboard.putNumber("leftMidF Velocity", leftMidFEncVelocity.toDouble())
        leftMidF.set(ControlMode.PercentOutput, 0.0)

        sleep(1000)
        leftMidR.set(ControlMode.PercentOutput, 1.0)
        sleep(1000)
        //Not sure if pidIdx if correct
        val leftMidREncPosition = leftMidR.getSelectedSensorPosition(0)
        val leftMidREncVelocity = leftMidR.getSelectedSensorVelocity(0)
        SmartDashboard.putNumber("leftMidR Position", leftMidREncPosition.toDouble())
        SmartDashboard.putNumber("leftMidR Velocity", leftMidREncVelocity.toDouble())
        leftMidR.set(ControlMode.PercentOutput, 0.0)

        sleep(1000)
        leftRear.set(ControlMode.PercentOutput, 1.0)
        sleep(1000)
        //Not sure if pidIdx if correct
        val leftRearEncPosition = leftRear.getSelectedSensorPosition(0)
        val leftRearEncVelocity = leftRear.getSelectedSensorVelocity(0)
        SmartDashboard.putNumber("leftRear Position", leftRearEncPosition.toDouble())
        SmartDashboard.putNumber("leftRear Velocity", leftRearEncVelocity.toDouble())
        leftRear.set(ControlMode.PercentOutput, 0.0)

        sleep(1000)
        rightFront.set(ControlMode.PercentOutput, 1.0)
        sleep(1000)
        //Not sure if pidIdx if correct
        val rightFrontEncPosition = rightFront.getSelectedSensorPosition(0)
        val rightFrontEncVelocity = rightFront.getSelectedSensorVelocity(0)
        SmartDashboard.putNumber("rightFront Position", rightFrontEncPosition.toDouble())
        SmartDashboard.putNumber("rightFront Velocity", rightFrontEncVelocity.toDouble())
        rightFront.set(ControlMode.PercentOutput, 0.0)

        sleep(1000)
        rightMidF.set(ControlMode.PercentOutput, 1.0)
        sleep(1000)
        //Not sure if pidIdx if correct
        val rightMidFEncPosition = rightMidF.getSelectedSensorPosition(0)
        val rightMidFEncVelocity = rightMidF.getSelectedSensorVelocity(0)
        SmartDashboard.putNumber("rightMidF Position", rightMidFEncPosition.toDouble())
        SmartDashboard.putNumber("rightMidF Velocity", rightMidFEncVelocity.toDouble())
        rightMidF.set(ControlMode.PercentOutput, 0.0)

        sleep(1000)
        rightMidR.set(ControlMode.PercentOutput, 1.0)
        sleep(1000)
        //Not sure if pidIdx if correct
        val rightMidREncPosition = rightMidR.getSelectedSensorPosition(0)
        val rightMidREncVelocity = rightMidR.getSelectedSensorVelocity(0)
        SmartDashboard.putNumber("rightMidR Position", rightMidREncPosition.toDouble())
        SmartDashboard.putNumber("rightMidR Velocity", rightMidREncVelocity.toDouble())
        rightMidR.set(ControlMode.PercentOutput, 0.0)

        sleep(1000)
        rightRear.set(ControlMode.PercentOutput, 1.0)
        sleep(1000)
        //Not sure if pidIdx if correct
        val rightRearEncPosition = rightRear.getSelectedSensorPosition(0)
        val rightRearEncVelocity = rightRear.getSelectedSensorVelocity(0)
        SmartDashboard.putNumber("rightRear Position", rightRearEncPosition.toDouble())
        SmartDashboard.putNumber("rightRear Velocity", rightRearEncVelocity.toDouble())
        rightRear.set(ControlMode.PercentOutput, 0.0)

        leftMidF.follow(leftFront)
        leftMidR.follow(leftFront)
        leftRear.follow(leftFront)

        rightMidF.follow(rightFront)
        rightMidR.follow(rightFront)
        rightRear.follow(rightFront)

        false
    }


    on (Events.TELEOP_ENABLED) {
        driveMachine.setState(DriveStates.OPEN_LOOP)
    }
}

