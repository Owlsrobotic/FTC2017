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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
public class DriverControl extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private double stickThreshold = 0.2;
    RobotController controller;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        controller = new RobotController(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        boolean isClawClosed = true;
        while (opModeIsActive()) {
            controller.leftBeltMotor.setPower(0.0);
            controller.rightBeltMotor.setPower(0.0);
            controller.trapdoorMotor.setPower(0.0);
            controller.elevatorMotor.setPower(0.0);

            //Emergency move jewel arm
            if (gamepad2.start) {
                controller.moveServo(controller.jewelsArm, 0.3);
            } else {
                controller.moveServo(controller.jewelsArm, 1.0);
            }

            //Toggle claw state
            if (gamepad2.a) {
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                isClawClosed = !isClawClosed;
            }
            //Rotate wrist
            if (gamepad2.right_bumper) {
                controller.moveServo(controller.wristServo, 1.0);
            }
            if (gamepad2.left_bumper) {
                controller.moveServo(controller.wristServo, 0.3);
            }
            //Manipulate elbow
            if (gamepad2.b) {
                controller.moveServo(controller.elbowServo, 0.6);
            }
            if (gamepad2.x) {
                controller.moveServo(controller.elbowServo, 0.1);
            }
            //Move elevator
            if (gamepad2.dpad_up) {
                controller.elevatorMotor.setPower(-0.6);
            }
            if (gamepad2.dpad_down) {
                controller.elevatorMotor.setPower(0.6);
            }
            //Move belts
            if (gamepad2.left_stick_y > stickThreshold || gamepad2.left_stick_y < -1.0 * stickThreshold) {
                controller.leftBeltMotor.setPower(-1.0 * gamepad2.left_stick_y);
            }
            if (gamepad2.right_stick_y > stickThreshold || gamepad2.right_stick_y < -1.0 * stickThreshold) {
                controller.rightBeltMotor.setPower(gamepad2.right_stick_y);
            }
            if (gamepad2.y) {
                controller.leftBeltMotor.setPower(0.7);
                controller.rightBeltMotor.setPower(-0.7);
            }

            //Open/Close claws
            if (isClawClosed) {
                controller.manipulateClaws(controller.CLAW_CLOSED);
            } else {
                controller.manipulateClaws(controller.CLAW_OPEN);
            }

           // trapdoor
            if(gamepad1.dpad_up) {
                controller.trapdoorMotor.setPower(0.7);
            }else if(gamepad1.dpad_down) {
                controller.trapdoorMotor.setPower(-0.7);
            }

            // translation
            if((gamepad1.left_stick_x > stickThreshold || gamepad1.left_stick_x < -1.0 * stickThreshold) ||
               (gamepad1.left_stick_y > stickThreshold || gamepad1.left_stick_y < -1.0 * stickThreshold)) {
                controller.moveDirection(0.5, -1 * gamepad1.left_stick_x, gamepad1.left_stick_y);
            } else {
                controller.move(0.0, controller.DIRECTION_FORWARD);
            }

            // rotation
            if((gamepad1.right_stick_x > stickThreshold) || (gamepad1.right_stick_x < -1.0 * stickThreshold)) {
                controller.rotate(gamepad1.right_stick_x, RobotController.ROTATE_RIGHT);
            }

            telemetry.update();
        }
    }
}
