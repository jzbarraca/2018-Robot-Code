package org.team401.robot2018

import org.snakeskin.ShifterState
import org.snakeskin.ShifterState.*
import org.team401.robot2018.Constants.DrivetrainParameters.DOWNSHIFT_CURRENT
import org.team401.robot2018.Constants.DrivetrainParameters.DELTA
import org.team401.robot2018.Constants.DrivetrainParameters.SPEED_THRESHOLD
import org.team401.robot2018.Constants.DrivetrainParameters.SPEED_SPLIT

object AutoShifter {
    private var lastShiftTime = System.currentTimeMillis()

    //currentTime and lastShiftTime expected in ms
    fun shiftAuto(currentTime: Long, currentAmpDraw: Double, currentVel: Double, currentGear: ShifterState): ShifterState {
        if (currentAmpDraw >= DOWNSHIFT_CURRENT) return LOW

        if (currentTime - lastShiftTime <= 250) return currentGear
        //if we have a high gear and our velocity is low, it probably means we're stuck against a wall
        //so we don't force the robot, we return a low gear
        if (currentGear == HIGH && currentVel < SPEED_THRESHOLD) return LOW

        if (SPEED_SPLIT - currentVel > DELTA) return LOW
        if (SPEED_SPLIT - currentVel < -DELTA) return HIGH
        //the velocity change is minor, so the gear ratio isn't changed
        return currentGear
    }

    fun update() {
        lastShiftTime = System.currentTimeMillis()
    }
}