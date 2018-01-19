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

@Autonomous(name="BlueCrypto", group="Blue")
public class AutonomousBlueCrypto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    RobotController controller;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        controller = new RobotController(this);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            controller.moveDistance(0.15, .2, controller.DIRECTION_REVERSE);

            //Move Jewel Arm into position
            controller.jewelsArm.setPosition(0.70);
            //Wait for arm to reach desired position
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            //Sensor is on the right of the arm...knock off red jewel if on blue side
            int color = controller.getColor(controller.jewelsColorSensor, 10);
            if(color == RobotController.COLOR_RED) {
                controller.moveDistance(0.2, 0.2, controller.DIRECTION_REVERSE);
                controller.moveDistance(0.2, 0.2, controller.DIRECTION_FORWARD);
            } else {
                controller.moveDistance(0.2, 0.2, controller.DIRECTION_FORWARD);
                controller.moveDistance(0.2, 0.2, controller.DIRECTION_REVERSE);
            }
            //Move Jewel Arm back up
            controller.jewelsArm.setPosition(0.0);

            //Move forward to see picture
            controller.moveDistance(0.5, 0.2, controller.DIRECTION_FORWARD);
            RelicRecoveryVuMark vuMark = controller.detectVumark();

            controller.moveDistance(0.5, 0.2, controller.DIRECTION_REVERSE);

            //Move to cryptobox
            controller.moveDistance(0.7, 0.2, controller.DIRECTION_REVERSE);
            if(vuMark == RelicRecoveryVuMark.LEFT) {
                controller.moveDistance(0.1, 0.1, controller.DIRECTION_LEFT);
            } else if(vuMark == RelicRecoveryVuMark.CENTER) {
                controller.moveDirection(0.3, 0.1, controller.DIRECTION_LEFT);
            } else {
                controller.moveDirection(0.5, 0.1, controller.DIRECTION_LEFT);
            }
            controller.moveDistance(0.4, 0.2, controller.DIRECTION_REVERSE);

            controller.leftBeltMotor.setPower(-0.2);
            controller.rightBeltMotor.setPower(0.2);
            break;
        }
    }
}
