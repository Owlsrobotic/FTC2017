package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Imu.ImuCalculator;
import org.firstinspires.ftc.teamcode.Imu.Orientation;

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

    public static int CLAW_CLOSED = 0;
    public static int CLAW_OPEN = 1;

    // no green as unneeded
    public static int COLOR_RED = 0;
    public static int COLOR_BLUE = 1;

    private LinearOpMode context;
    private HardwareMap hmap;

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    // Todo: Implement motor bindings
    DcMotor trapdoorMotor;
    DcMotor elevatorMotor;
    DcMotor leftBeltMotor;
    DcMotor rightBeltMotor;

    // Todo: Implement servo bindings
    Servo leftClaw;
    Servo rightClaw;
    Servo wristServo;
    Servo elbowServo;

    //Vuforia fields
    int cameraMonitorViewId;
    VuforiaLocalizer.Parameters parameters;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;

    BNO055IMU imu;
//    Thread imuThread;
    ImuCalculator imuCalc;

    // autonomous
    DigitalChannel digitalTouch;
    DistanceSensor distanceSensor;

    // Jewels Control
    public Servo jewelsArm;
    public ColorSensor jewelsColorSensor;

    public RobotController(LinearOpMode context){
        this.context = context;

        hmap = context.hardwareMap;

        //Setting up motors
        frontRight = hmap.dcMotor.get("front_right");
        frontLeft = hmap.dcMotor.get("front_left");
        backRight = hmap.dcMotor.get("back_right");
        backLeft = hmap.dcMotor.get("back_left");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Setting up servos
        leftClaw = hmap.servo.get("left_claw");
        rightClaw = hmap.servo.get("right_claw");
        elbowServo = hmap.servo.get("elbow");
        wristServo = hmap.servo.get("wrist");

        trapdoorMotor = hmap.dcMotor.get("trapdoor");
        elevatorMotor = hmap.dcMotor.get("elevator");
        leftBeltMotor = hmap.dcMotor.get("left_belt");
        rightBeltMotor = hmap.dcMotor.get("right_belt");

        // autonomous stuff
//        digitalTouch = hmap.get(DigitalChannel.class, "test_touch");
//        distanceSensor = hmap.get(DistanceSensor.class, "test_distance");

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



//        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        jewelsArm = hmap.servo.get("jewels_arm");
        jewelsColorSensor = hmap.colorSensor.get("test_color");
    }

    // angle is in radians
    public void rotateAngle(double angle, double power){
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


    frontLeft.setTargetPosition(newFrontLeftTarget);
    backLeft.setTargetPosition(newBackLeftTarget);
    frontRight.setTargetPosition(newFrontRightTarget);
    backRight.setTargetPosition(newBackRightTarget);
	//NOTE: Left side has reversed polarity
	if(angle > 0){
		//Rotate Clockwise: Left side moves foward while right side moves backward
        frontLeft.setPower(-1 * power);
        backLeft.setPower(-1 * power);
        frontRight.setPower(-1 * power);
        backRight.setPower(-1 * power);
	}else{
		//Rotate Counter-Clockwise: Right side moves foward while left side moves backward
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
	}

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

    public void move(double power, int direction) {
        //Front Left and Back Left have reversed polarity ... use negative for forward

        if (direction == DIRECTION_FORWARD) {
            frontRight.setPower(power);
            frontLeft.setPower(-1 * power);
            backRight.setPower(power);
            backLeft.setPower(-1 * power);
        }
        if (direction == DIRECTION_REVERSE) {
            frontRight.setPower(-1 * power);
            frontLeft.setPower(power);
            backRight.setPower(-1 * power);
            backLeft.setPower(power);
        }
        if (direction == DIRECTION_RIGHT) {
            frontRight.setPower(-1 * power);
            frontLeft.setPower(-1 * power);
            backRight.setPower(power);
            backLeft.setPower(power);
        }
        if (direction == DIRECTION_LEFT) {
            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(-1 * power);
            backLeft.setPower(-1 * power);
        }
    }

    public void moveDirection(double power, double directionX, double directionY) {
        double magnitude = Math.sqrt(Math.pow(directionX, 2) + Math.pow(directionY, 2));
        double normalizedX = directionX / magnitude;
        double normalizedY = directionY / magnitude;

        double frontRightFow = power * magnitude;
        double frontLeftFow = -1.0 * power * magnitude;
        double backRightFow = power * magnitude;
        double backLeftFow = -1.0 * power * magnitude;

        double frontRightRight = -1.0 * power * magnitude;
        double frontLeftRight = -1.0 * power * magnitude;
        double backRightRight = power * magnitude;
        double backLeftRight = power * magnitude;

        frontRight.setPower(normalizedY * frontRightFow + normalizedX * frontRightRight);
        frontLeft.setPower(normalizedY * frontLeftFow + normalizedX * frontLeftRight);
        backRight.setPower(normalizedY * backRightFow + normalizedX * backRightRight);
        backLeft.setPower(normalizedY * backLeftFow + normalizedX * backLeftRight);
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

    public int getRawColor(ColorSensor sensor, int color) {
        if (color == COLOR_RED) {
            return sensor.red();
        } else if (color == COLOR_BLUE) {
            return sensor.blue();
        } else {
            return 2;
        }
    }

    public int getColor(ColorSensor sensor, int numberOfScans) {

        int completedScans = 0;
        int totalRed = 0;
        int totalBlue = 0;

        while (completedScans < numberOfScans) {

            int rawColor = rawColorScan(sensor);

            while (rawColor == 2) {
                rawColor = rawColorScan(sensor);
            }

            if (rawColor == COLOR_RED) {
                totalRed++;
            } else if (rawColor == COLOR_BLUE) {
                totalBlue++;
            }

            completedScans++;
        }

        int totalColor = totalColorScan(totalRed, totalBlue);

        while (totalColor == 2) {
            totalColor = totalColorScan(totalRed, totalBlue);
        }

        if (totalColor == COLOR_RED) {
            return COLOR_RED;
        } else if (totalColor == COLOR_BLUE) {
            return COLOR_BLUE;
        }

        // impossible to reach
        return 2;
    }

    private int rawColorScan(ColorSensor sensor) {

        int amountRed = getRawColor(sensor, COLOR_RED);
        int amountBlue = getRawColor(sensor, COLOR_BLUE);

        if (amountRed > amountBlue) {
            return COLOR_RED;
        } else if (amountBlue > amountRed){
            return COLOR_BLUE;
        }

        // one in a million chance, but it's not hard to fix
        return 2;
    }

    private int totalColorScan(int totalRed, int totalBlue) {
        if (totalRed > totalBlue) {
            return COLOR_RED;
        } else if (totalBlue > totalRed)
            return COLOR_BLUE;

        // one in a million chance, but it's not hard to fix
        return 2;
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

    public void manipulateClaws(int position) {
        if(position == CLAW_CLOSED) {
            moveServo(leftClaw, 1.0);
            moveServo(rightClaw, 0.0);
        } else if (position == CLAW_OPEN) {
            moveServo(leftClaw, .4);
            moveServo(rightClaw, .7);
        }
    }
}
