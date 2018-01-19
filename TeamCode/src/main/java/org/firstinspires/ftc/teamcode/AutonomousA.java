/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="wizard", group="Linear Opmode")
public class AutonomousA extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    RobotController controller;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        controller = new RobotController(this);

        // wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // make sure this shit can read
        controller.jewelsColorSensor.enableLed(true);

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {
            try {
                // image recognition
                controller.rotateAngle(Math.toRadians(40), 0.55);
                RelicRecoveryVuMark mark = controller.detectVumark();
                Thread.sleep(2100);
                controller.rotateAngle(Math.toRadians(-40), 0.55);
                telemetry.addData("VUMARK: ", mark.toString());
                telemetry.update();

                // align for blunt
                controller.rotateAngle(Math.toRadians(-23), 0.55);

                // color sensor
                controller.moveServo(controller.jewelsArm, 0.0);

                Thread.sleep(2000);

                if (controller.getColor(controller.jewelsColorSensor, 10) == controller.COLOR_RED) {
                    controller.rotateAngle(Math.toRadians(-25), 0.55);
                    Thread.sleep(2000);
                    controller.moveServo(controller.jewelsArm, 1.0);
                    Thread.sleep(2000);
                    controller.rotateAngle(Math.toRadians(25), 0.55);
                } else {
                    controller.rotateAngle(Math.toRadians(-25), 0.55);
                    Thread.sleep(2000);
                    controller.moveServo(controller.jewelsArm, 1);
                    Thread.sleep(2000);
                    controller.rotateAngle(Math.toRadians(25), 0.55);
                }

                // get bot's ass back in position
                controller.rotateAngle(Math.toRadians(23), 0.55);
                Thread.sleep(2000);

                // do each case and shit
                switch (mark) {
                    case LEFT:
                        controller.moveDistance(0.71, 0.1, controller.DIRECTION_FORWARD);
                        break;
                    case CENTER:
                        controller.moveDistance(0.89, 0.1, controller.DIRECTION_FORWARD);
                        break;
                    case RIGHT:
                        controller.moveDistance(1.0, 0.1, controller.DIRECTION_FORWARD);
                        break;
                }

                // lemme slide in bruh
                Thread.sleep(2000);
                controller.rotateAngle(Math.toRadians(90), 0.3);
                Thread.sleep(2000);
                controller.moveDistance(0.25, 0.5, controller.DIRECTION_FORWARD);

                // push that shit homie
                controller.leftBeltMotor.setPower(0.5);
                controller.rightBeltMotor.setPower(-0.5);

                Thread.sleep(2000);

                // stop conveyor
                controller.leftBeltMotor.setPower(0);
                controller.rightBeltMotor.setPower(0);
            }

            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }
}
