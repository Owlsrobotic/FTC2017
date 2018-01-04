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
        while (opModeIsActive()) {

            // jewel recognition
            controller.moveServo(controller.jewelsArm, 0.56);
            //Wait for servo to get into position then get color reading
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            String color = controller.getColor(controller.jewelsColorSensor, 10);


            if(color=="blue") {
                controller.moveDistance(0.1, 0.1, controller.DIRECTION_REVERSE);
                controller.moveServo(controller.jewelsArm, 0);
                controller.moveDistance(-0.1, 0.1, controller.DIRECTION_REVERSE);
            } else {
                controller.moveDistance(-0.1, 0.1, controller.DIRECTION_REVERSE);
                controller.moveServo(controller.jewelsArm, 0);
                controller.moveDistance(0.1, 0.1, controller.DIRECTION_REVERSE);
            }

            // image recognition
            controller.moveDistance(.4, .1, controller.DIRECTION_FORWARD);
            RelicRecoveryVuMark mark = controller.detectVumark();
            telemetry.addData("VUMARK: ", mark.toString());
            telemetry.update();
            switch (mark) {
                case LEFT:
                    controller.moveDistance(.5, .1, controller.DIRECTION_LEFT);
                    break;
                case CENTER:
                    controller.rotate(Math.toRadians(45), .5);
                    break;
                case RIGHT:
                    controller.rotate(Math.toRadians(-45), .5);
                    break;
            }
            break;
            /*while(runtime.time(TimeUnit.SECONDS) < 5) {
                telemetry.addData("blue", telemetry.addData("blue", controller.testColorSensor.blue()));
                telemetry.addData("green", controller.testColorSensor.green());
                telemetry.addData("red", controller.testColorSensor.red());
                telemetry.update();
            }

            controller.move(0.15, DIRECTION_FORWARD);


            for(int i=0; i<1; i++) {
                while (controller.getDistance(controller.testDistanceSensor, DistanceUnit.CM) > 10 || Double.isNaN(controller.getDistance(controller.testDistanceSensor, DistanceUnit.CM))) {
                    telemetry.addData("Distance: ", controller.getDistance(controller.testDistanceSensor, DistanceUnit.CM));
                    telemetry.update();
                }
            }

            controller.move(0, DIRECTION_FORWARD);

            if(controller.getColor(controller.testColorSensor, 10) == "blue") {
                telemetry.addData("blue", controller.testColorSensor.blue());
                controller.moveDistance(0.3, 1.0, DIRECTION_LEFT);
            }
            else if(controller.getColor(controller.testColorSensor, 10) == "red") {
                controller.moveDistance(0.3, 1.0, DIRECTION_RIGHT);
            }
            else if(controller.getColor(controller.testColorSensor, 10) == "green") {
                controller.moveDistance(0.3, 1.0, DIRECTION_REVERSE);
            }

            controller.move(0.4, DIRECTION_FORWARD);

            for(int i=0; i<1; i++) {
                while (controller.digitalTouch.getState() == true) { // inverse because whoever designed this is stupid
                    telemetry.addData("Distance: ", controller.getDistance(controller.testDistanceSensor, DistanceUnit.CM));
                    telemetry.addData("State: ", controller.digitalTouch.getState());
                    telemetry.update();
                }
            }

            controller.move(0, DIRECTION_FORWARD);*/
        }
    }
}
