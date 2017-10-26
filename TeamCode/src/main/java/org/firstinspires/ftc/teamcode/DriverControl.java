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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    RobotController controller;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        controller = new RobotController(this);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        DcMotor fl = hardwareMap.dcMotor.get("front_left");
        DcMotor fr = hardwareMap.dcMotor.get("front_right");
        DcMotor bl = hardwareMap.dcMotor.get("back_left");
        DcMotor br = hardwareMap.dcMotor.get("back_right");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            controller.detectVumark();

            if(gamepad1.y){
                controller.moveDistance(1.7, .5, RobotController.DIRECTION_FOWARD);
            }else if(gamepad1.x){
                controller.moveDistance(1.7, .5, RobotController.DIRECTION_LEFT);
            }else if(gamepad1.b){
                controller.moveDistance(1.7, .5, RobotController.DIRECTION_RIGHT);
            }else if(gamepad1.a){
                controller.moveDistance(1.7, .5, RobotController.DIRECTION_REVERSE);
            }

            if(gamepad1.dpad_right){
                controller.rotate(Math.toRadians(45), .7);
            }
            /*}else if(gamepad1.right_bumper){
                controller.rotate(.7, RobotController.ROTATE_RIGHT);
            }else if(gamepad1.left_bumper){
                controller.rotate(.7, RobotController.ROTATE_LEFT);
            }*/

            if((gamepad1.right_stick_x > 0.1) || (gamepad1.right_stick_x < -0.1)) {
                if (gamepad1.right_stick_x > 0) {
                    controller.rotate(gamepad1.right_stick_x, RobotController.ROTATE_RIGHT);
                } else if (gamepad1.right_stick_x < 0) {
                    controller.rotate(-1 * gamepad1.right_stick_x, RobotController.ROTATE_LEFT);
                }
            }

            if((gamepad1.left_stick_x > 0.1) || (gamepad1.left_stick_x < -0.1)) {
                if (gamepad1.left_stick_x > 0) {
                    controller.move(-1 * gamepad1.left_stick_x, RobotController.DIRECTION_LEFT);
                } else if (gamepad1.left_stick_x < 0) {
                    controller.move(gamepad1.left_stick_x, RobotController.DIRECTION_RIGHT);
                }
            }

            if((gamepad1.left_stick_y > 0.1) || (gamepad1.left_stick_y < -0.1)) {
                if (gamepad1.left_stick_y > 0) {
                    controller.move(-1 * gamepad1.left_stick_y, RobotController.DIRECTION_FOWARD);
                } else if (gamepad1.left_stick_y < 0) {
                    controller.move(gamepad1.left_stick_y, RobotController.DIRECTION_REVERSE);
                }
            }

            controller.move(0, RobotController.DIRECTION_FOWARD);

            telemetry.addData("handome lakewe: ", gamepad1.right_stick_x);
            telemetry.addData("Imu Calc", controller.getRobotOrientation().toString());
            telemetry.update();
        }
    }
}
