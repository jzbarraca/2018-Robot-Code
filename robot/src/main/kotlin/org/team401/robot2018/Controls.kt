package org.team401.robot2018

import org.snakeskin.dsl.HumanControls
import org.snakeskin.dsl.machine
import org.team401.robot2018.subsystems.MotionMagicTestSubsystem
import org.team401.robot2018.subsystems.position

val Stick = HumanControls.attack3(0){

    invertAxis(Axes.PITCH)

    whenButton(Buttons.TRIGGER){
        pressed {
            MotionMagicTestSubsystem.machine().toggle("Open loop", "TEST")
        }
    }
    whenButton(3){
        pressed {
            position += 1024.0
        }
    }
    whenButton(2){
        pressed {
            position -= 1024.0
        }
    }
}