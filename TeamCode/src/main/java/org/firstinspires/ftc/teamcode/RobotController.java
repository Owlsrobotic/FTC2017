package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Imu.ImuCalculator;
import org.firstinspires.ftc.teamcode.Imu.Orientation;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * RobotController controls all hardware on the robot
 * and abstracts all computation done by the robot
 */

public class RobotController {

    // wheel configuration
    double     COUNTS_PER_MOTOR_REV    = 1120 ;
    double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    double     WHEEL_DIAMETER_METERS   = 0.1016 ;
    double     COUNTS_PER_METER        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_METERS * 3.1415);

    double ROTATION_RADIUS = .464 / 2;

    // enumeration declarations
    public static int DIRECTION_FORWARD = 0;
    public static int DIRECTION_REVERSE = 1;
    public static int DIRECTION_LEFT = 2;
    public static int DIRECTION_RIGHT = 3;

    public static int ROTATE_LEFT = 4;
    public static int ROTATE_RIGHT = 5;

    private LinearOpMode context;
    private HardwareMap hmap;

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    //Vuforia fields
    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters parameters;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    BNO055IMU imu;
//    Thread imuThread;
    ImuCalculator imuCalc;

    // autonomous tests
    ColorSensor testColorSensor;
    DistanceSensor testDistanceSensor;
    TouchSensor testTouchSensor;

    public Servo jewelsArm;

    public RobotController(LinearOpMode context){
        this.context = context;

        hmap = context.hardwareMap;

        //Setting up motors
        frontRight = hmap.dcMotor.get("front_right");
        frontLeft = hmap.dcMotor.get("front_left");
        backRight = hmap.dcMotor.get("back_right");
        backLeft = hmap.dcMotor.get("back_left");

        //Setting up Vuforia
        cameraMonitorViewId = hmap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hmap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AQfcvgf/////AAAAGdN+qKpBsEVHoQv9Tbl9rEwid5sqZQ/PU7AaS3noa/ht7K7vgAgjR7jdMqq3uv5CZR5l98sacbhIfOjgnl6IAVNMfmbge4tSJcjida+/BmwDbq0QfsTYfsPzQE+86L0xgY7gRnnU++Voak+/mNpFrGAMR66VHgWP1nxM4PqdoqTwAnezheyd35NczIRFBTReI3p3HbbIJiXBmriFO8YMBC0B5/ZD1Euh8yXhj0cTFDR0T4Evjp1bNHOQkiZkFYzg12gpLgiWDJavXhOnb7dUHhKlLzVEZQ/aRaW0YzYJPZQzRE0CF0vo9tZ4EUaCvG1ieBFtu88ZXHJGGsYG5TxJb6iGLzR3iOfNIKhfMZ2az6PC";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();

        //Setting up IMU
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = true;


//        imu = hmap.get(BNO055IMU.class, "imu");
//        imu.initialize(imuParameters);

//        imu.startAccelerationIntegration(new Position(), new Velocity(), 2000);

//        imuCalc = new ImuCalculator(imu, context);
//        imuThread = new Thread(imuCalc);
//        imuThread.start();

        testColorSensor = hmap.colorSensor.get("test_color");
        testDistanceSensor = hmap.get(DistanceSensor.class, "test_distance");

        jewelsArm = hmap.servo.get("jewels_arm");
    }

    // angle is in radians
    public void rotate(double angle, double power){
        double distance = ROTATION_RADIUS * angle;

        // reset all motors
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // tell to run encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_METER);
        int newFrontRightTarget = frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_METER);
        int newBackLeftTarget = backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_METER);
        int newBackRightTarget = backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_METER);


        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

	//NOTE: Left side has reversed polarity
	if(angle > 0){
		//Rotate Clockwise: Left side moves foward while right side moves backward
		frontLeft.setTargetPosition(-1 * newFrontLeftTarget);
        	backLeft.setTargetPosition(-1 * newBackLeftTarget);
        	frontRight.setTargetPosition(-1 * newFrontRightTarget);
        	backRight.setTargetPosition(-1 * newBackRightTarget);
	}else{
		//Rotate Counter-Clockwise: Right side moves foward while left side moves backward
		frontLeft.setTargetPosition(newFrontLeftTarget);
        	backLeft.setTargetPosition(newBackLeftTarget);
        	frontRight.setTargetPosition(newFrontRightTarget);
        	backRight.setTargetPosition(newBackRightTarget);
	}
       
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        while(frontRight.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy()){
           //Do nothing while busy
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        //Turn off run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveDistance(double distance, double power, int direction){
        //Reset all motors
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Tell to run encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_METER);
        int newFrontRightTarget = frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_METER);
        int newBackLeftTarget = backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_METER);
        int newBackRightTarget = backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_METER);

        int test = -1;

        if(direction == DIRECTION_FORWARD){
            test = direction;

            frontLeft.setTargetPosition(-1 * newFrontLeftTarget);
            backLeft.setTargetPosition(-1 * newBackLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);
        }
        if(direction == DIRECTION_REVERSE){
            test = direction;

            frontLeft.setTargetPosition(newFrontLeftTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            frontRight.setTargetPosition(-1 * newFrontRightTarget);
            backRight.setTargetPosition(-1 * newBackRightTarget);
        }
        if(direction == DIRECTION_LEFT){
            test = direction;

            frontLeft.setTargetPosition(newFrontLeftTarget);
            backLeft.setTargetPosition(-1 * newBackLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(-1 * newBackRightTarget);
        }
        if(direction == DIRECTION_RIGHT){
            test = direction;

            frontLeft.setTargetPosition(-1 * newFrontLeftTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            frontRight.setTargetPosition(-1 * newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);

        int i = 0;
        while(frontRight.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy()){
	    //Telemetry
            context.telemetry.addData("frontLeft", frontLeft.isBusy());
            context.telemetry.addData("frontRight", frontRight.isBusy());
            context.telemetry.addData("backLeft", backLeft.isBusy());
            context.telemetry.addData("backRight", backRight.isBusy());
            context.telemetry.addData("direction", test);

            context.telemetry.addData("Iteration", ++i);


            context.telemetry.update();
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        //Turn off run to position
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void move(double power, int direction){
        //Front Left and Back Left have reversed polarity ... use negative for forward

        if(direction == DIRECTION_FORWARD){
            frontRight.setPower(power);
            frontLeft.setPower(-1 * power);
            backRight.setPower(power);
            backLeft.setPower(-1 * power);
        }
        if(direction == DIRECTION_REVERSE){
            frontRight.setPower(-1 * power);
            frontLeft.setPower(power);
            backRight.setPower(-1 * power);
            backLeft.setPower(power);
        }
        if(direction == DIRECTION_RIGHT){
            frontRight.setPower(-1 * power);
            frontLeft.setPower(-1 * power);
            backRight.setPower(power);
            backLeft.setPower(power);
        }
        if(direction == DIRECTION_LEFT){
            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(-1 * power);
            backLeft.setPower(-1 * power);
        }
    }

    public void rotate(double power, int direction){
        if(direction == RobotController.ROTATE_LEFT){
            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);
        }
        if(direction == RobotController.ROTATE_RIGHT){
            frontRight.setPower(-1 * power);
            frontLeft.setPower(-1 * power);
            backRight.setPower(-1 * power);
            backLeft.setPower(-1 * power);
        }
    }

    public RelicRecoveryVuMark detectVumark(){
        RelicRecoveryVuMark result = RelicRecoveryVuMark.from(relicTemplate);
        context.telemetry.addData("Vumark", result.toString());
//        context.telemetry.addData("Vumark license", hmap.appContext.getString(R.string.vuforia_license));
        context.telemetry.update();
        return result;
    }

    public Orientation getRobotOrientation() {
        return imuCalc.getOrientation();
//        Position pos = imu.getPosition();
//        return new Orientation(pos.x, pos.y, 0);
    }

    public int getRawColor(ColorSensor sensor, String color) {
        if (color == "red") {
            return sensor.red();
        }
        if (color == "green") {
            return sensor.green();
        }
        if (color == "blue") {
            return sensor.blue();
        } else {
            return 0;
        }
    }

    public String getColor(ColorSensor sensor, int numberOfScans) {

        int completedScans = 0;

        int totalRed = 0;
        int totalGreen = 0;
        int totalBlue = 0;

        while (completedScans < numberOfScans) {

            String rawColor = rawColorScan(sensor);

            while (rawColor == "error") {
                rawColor = rawColorScan(sensor);
            }

            if (rawColor == "red") {
                totalRed++;
            } else if (rawColor == "green") {
                totalGreen++;
            } else if (rawColor == "blue") {
                totalBlue++;
            }

            completedScans++;
        }

        String totalColor = totalColorScan(sensor, totalRed, totalGreen, totalBlue);

        while (totalColor == "error") {
            totalColor = totalColorScan(sensor, totalRed, totalGreen, totalBlue);
        }

        if (totalColor == "red") {
            return "red";
        } else if (totalColor == "green") {
            return "green";
        } else if (totalColor == "blue") {
            return "blue";
        }

        return "error";
    }

    private String rawColorScan(ColorSensor sensor) {

        int amountRed = getRawColor(sensor, "red");
        int amountGreen = getRawColor(sensor, "green");
        int amountBlue = getRawColor(sensor, "blue");

        if (amountRed > amountGreen && amountRed > amountBlue) {
            return "red";
        } else if (amountGreen > amountRed && amountGreen > amountBlue) {
            return "green";
        } else if (amountBlue > amountRed && amountBlue > amountGreen) {
            return "blue";
        }

        return "error";
    }

    private String totalColorScan(ColorSensor sensor, int totalRed, int totalGreen, int totalBlue) {

        if (totalRed > totalBlue && totalRed > totalGreen) {
            return "red";
        } else if (totalGreen > totalRed && totalGreen > totalBlue) {
            return "green";
        } else if (totalBlue > totalRed && totalBlue > totalGreen) {
            return "blue";
        }

        return "error";
    }

    public double getDistance(DistanceSensor sensor, DistanceUnit unit) {
        double distance = sensor.getDistance(unit);

        return distance;
    }

    // position is on range from 0 to 1.0 where 1.0 will move to maximum position
    // position can be negative to reverse direction
    public void moveServo(Servo servo, double position) {
        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(position);
    }
}
