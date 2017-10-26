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

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.android.dex.util.ExceptionWithContext;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;


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

    //Vuforia fields
    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters parameters;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQfcvgf/////AAAAGdN+qKpBsEVHoQv9Tbl9rEwid5sqZQ/PU7AaS3noa/ht7K7vgAgjR7jdMqq3uv5CZR5l98sacbhIfOjgnl6IAVNMfmbge4tSJcjida+/BmwDbq0QfsTYfsPzQE+86L0xgY7gRnnU++Voak+/mNpFrGAMR66VHgWP1nxM4PqdoqTwAnezheyd35NczIRFBTReI3p3HbbIJiXBmriFO8YMBC0B5/ZD1Euh8yXhj0cTFDR0T4Evjp1bNHOQkiZkFYzg12gpLgiWDJavXhOnb7dUHhKlLzVEZQ/aRaW0YzYJPZQzRE0CF0vo9tZ4EUaCvG1ieBFtu88ZXHJGGsYG5TxJb6iGLzR3iOfNIKhfMZ2az6PC";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = true;

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        RobotController controller = new RobotController(this);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            RelicRecoveryVuMark result = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("Vumark", result.toString());
//            context.telemetry.addData("Vumark license", hmap.appContext.getString(R.string.vuforia_license));

            telemetry.addData("AccelX", imu.getLinearAcceleration().xAccel);
            telemetry.addData("AccelY", imu.getLinearAcceleration().yAccel);
            telemetry.addData("AccelZ", imu.getLinearAcceleration().zAccel);
            telemetry.addData("GyroX", imu.getAngularVelocity().xRotationRate);
            telemetry.addData("GyroY", imu.getAngularVelocity().yRotationRate);
            telemetry.addData("GyroZ", imu.getAngularVelocity().zRotationRate);

            telemetry.update();

//            if(gamepad1.y){
//                controller.moveDistance(1.7, .5, RobotController.DIRECTION_FOWARD);
//            }else if(gamepad1.x){
//                controller.moveDistance(1.7, .5, RobotController.DIRECTION_LEFT);
//            }else if(gamepad1.b){
//                controller.moveDistance(1.7, .5, RobotController.DIRECTION_RIGHT);
//            }else if(gamepad1.a){
//                controller.moveDistance(1.7, .5, RobotController.DIRECTION_REVERSE);
//            }
        }
    }
}
