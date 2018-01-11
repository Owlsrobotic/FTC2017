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

@TeleOp(name="Tester", group="Linear Opmode")
public class Tester extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
       // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        RobotController controller = new RobotController(this);

        // run until the end of the match (driver presses STOP)
        boolean isClawClosed = false;
        while (opModeIsActive()) {
            controller.move(0.0, controller.DIRECTION_FORWARD);
            controller.leftBeltMotor.setPower(0.0);
            controller.rightBeltMotor.setPower(0.0);
            controller.trapdoorMotor.setPower(0.0);
            controller.elevatorMotor.setPower(0.0);

            if (gamepad1.b) {
                controller.move(1.0, controller.DIRECTION_FORWARD);
            }
            if (gamepad1.a) {
                controller.leftBeltMotor.setPower(0.7);
                controller.rightBeltMotor.setPower(-0.7);
            }
            if (gamepad1.y){
                controller.trapdoorMotor.setPower(0.7);
            }
            if (gamepad1.x) {
                controller.trapdoorMotor.setPower(-0.7);
            }
            if (gamepad1.dpad_up) {
                controller.elevatorMotor.setPower(-0.6);
            }
            if (gamepad1.dpad_down) {
                controller.elevatorMotor.setPower(0.6);
            }
            //Rotate wrist
            if (gamepad1.right_bumper) {
                controller.moveServo(controller.wristServo, 1.0);
            }
            if (gamepad1.left_bumper) {
                controller.moveServo(controller.wristServo, 0.3);
            }
            if (gamepad1.dpad_right) {
                controller.moveServo(controller.elbowServo, 1.0);
            }
            if (gamepad1.dpad_left) {
                controller.moveServo(controller.elbowServo, 0.0);
            }

            //Toggle claw state
            if (gamepad1.start) {
                isClawClosed = !isClawClosed;
            }
            //Open/Close claws
            if (isClawClosed) {
                controller.manipulateClaws(controller.CLAW_CLOSED);
            } else {
                controller.manipulateClaws(controller.CLAW_OPEN);
            }

            if (gamepad2.dpad_up) {
                controller.moveDistance(1.0, 0.7, controller.DIRECTION_FORWARD);
            }
            if (gamepad2.dpad_down) {
                controller.moveDistance(1.0, 0.7, controller.DIRECTION_REVERSE);
            }
            if (gamepad2.dpad_right) {
                controller.moveDistance(1.0, 0.7, controller.DIRECTION_RIGHT);
            }
            if (gamepad2.dpad_left) {
                controller.moveDistance(1.0, 0.7, controller.DIRECTION_LEFT);
            }
            if (gamepad2.right_bumper) {
                controller.rotate(Math.toRadians(90), 0.7);
            }
            if (gamepad2.left_bumper) {
                controller.rotate(Math.toRadians(-90), 0.7);
            }

        }
    }
}
