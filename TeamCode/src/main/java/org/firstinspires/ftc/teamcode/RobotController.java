package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * RobotController controls all hardware on the robot
 * and abstracts all computation done by the robot
 */

public class RobotController {

    //Wheel Configuration
    double     COUNTS_PER_MOTOR_REV    = 1120 ;
    double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    double     WHEEL_DIAMETER_METERS   = 0.1016 ;
    double     COUNTS_PER_METER        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_METERS * 3.1415);

    double ROTATION_RADIUS = 1.0;

    //Enumeration Declarations
    public static int DIRECTION_FOWARD = 0;
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

    public RobotController(LinearOpMode context){
        this.context = context;

        hmap = context.hardwareMap;

        frontRight = hmap.dcMotor.get("front_right");
        frontLeft = hmap.dcMotor.get("front_left");
        backRight = hmap.dcMotor.get("back_right");
        backLeft = hmap.dcMotor.get("back_left");
    }

    //angle is in radian
    public void rotate(double angle, double power){
        double distance = ROTATION_RADIUS * angle;

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

        if(direction == DIRECTION_FOWARD){
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

        if(direction == DIRECTION_FOWARD){
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
}
