package org.team401.robot2018.subsystems

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Talon
import org.snakeskin.dsl.*;
import org.snakeskin.event.Events
import org.snakeskin.logic.LockingDelegate
import org.team401.robot2018.Stick


var position by LockingDelegate(0.0)

val MotionMagicTestSubsystem: Subsystem = buildSubsystem {
    val talon = TalonSRX(6)


    setup {
        talon.setSelectedSensorPosition(0,0,0)
        talon.config_kF(0, 1023/932.0, 0)
        talon.config_kP(0,0.5,0)
        talon.config_kI(0,0.0,0)
        talon.config_kD(0,0.0,0)
        talon.configMotionCruiseVelocity(0, 0)
        talon.configMotionAcceleration(0,0)
    }
    val machine = stateMachine("MotionMagic"){

        state("TEST"){
            entry {
                talon.configMotionCruiseVelocity(700, 0)
                talon.configMotionAcceleration(450, 0)
            }
            action{
                //MAX Velocity = 932
                talon.set(ControlMode.MotionMagic, position)

            }

        }

        state("Open loop"){
            action {
                talon.set(ControlMode.PercentOutput, Stick.readAxis { PITCH })
                println("${talon.getSelectedSensorVelocity(0)*(600.0/4096)} ${System.currentTimeMillis()}")
            }
        }

    }

    on(Events.TELEOP_ENABLED){
        machine.setState("Open loop")
    }
}