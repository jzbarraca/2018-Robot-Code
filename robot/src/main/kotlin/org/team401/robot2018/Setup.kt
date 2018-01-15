package org.team401.robot2018

import org.snakeskin.auto.TempAutoManager
import org.snakeskin.dsl.*
import org.snakeskin.registry.*
import org.team401.robot2018.subsystems.MotionMagicTestSubsystem

/*
 * 2018-Robot-Code - Created on 1/5/18
 * Author: Cameron Earle
 * 
 * This code is licensed under the GNU GPL v3
 * You can find more info in the LICENSE file at project root
 */

/**
 * @author Cameron Earle
 * @version 1/5/18
 */

@Setup fun setup() {
    Subsystems.add(MotionMagicTestSubsystem) //TODO Add subsystems
    Controllers.add(Stick) //TODO Add controllers
    TempAutoManager.auto = autoLoop {  }
}