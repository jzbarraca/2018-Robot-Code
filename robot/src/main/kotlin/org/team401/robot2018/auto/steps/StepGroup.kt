package org.team401.robot2018.auto.steps

/*
 * 2018-Robot-Code - Created on 1/25/18
 * Author: Cameron Earle
 * 
 * This code is licensed under the GNU GPL v3
 * You can find more info in the LICENSE file at project root
 */

/**
 * @author Cameron Earle
 * @version 1/25/18
 */

class StepGroup(): AutoStep() {
    constructor(stepsIn: List<AutoStep>) : this() {
        steps.addAll(stepsIn)
    }

    constructor(vararg stepsIn: AutoStep) : this() {
        steps.addAll(stepsIn)
    }

    private val steps = arrayListOf<AutoStep>()

    override fun entry() {
        steps.forEach {
            it.entry()
        }
    }

    override fun action() {
        steps.forEach {
            if (!it.done) {
                it.action()
            }
        }
        if (steps.all { it.done }) {
            done = true
        }
    }

    override fun exit() {
        steps.forEach {
            it.exit()
        }
    }
}