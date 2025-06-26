package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class Teleop_good extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    CRServo servoBucket;
    CRServo servoPinceR;
    CRServo servoPinceL;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor armMotor;
    DcMotor slideMotor;
    DcMotor intakeMotor;

    enum StateMachine {
        WAITING_FOR_TARGET,
        DRIVE_TO_TARGET_A
    }

    static final Pose2D TARGET_A = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);


    private static final int POSITION_1 = -375; // Preset position 1 (encoder counts) 800
    private static final int POSITION_2 = -3200;// 3000


    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.dcMotor.get("fl");
        backLeftMotor = hardwareMap.dcMotor.get("bl");
        frontRightMotor = hardwareMap.dcMotor.get("fr");
        backRightMotor = hardwareMap.dcMotor.get("br");
        armMotor = hardwareMap.dcMotor.get("arm");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        slideMotor = hardwareMap.get(DcMotor.class, "ele");
        servoBucket = hardwareMap.get(CRServo.class, "servoBucket");
        servoPinceR = hardwareMap.get(CRServo.class, "servoPinceR");
        servoPinceL = hardwareMap.get(CRServo.class, "servoPinceL");



        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        imu.initialize(parameters);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(80.0, -55); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_TARGET;

        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();

        slideMotor.setTargetPosition(POSITION_1);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.5);
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            telemetry.update();
            odo.update();
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.7;

            double up = (double) (gamepad1.dpad_up ? 1 : 0) /2;
            double down = (double) (gamepad1.dpad_down ? 1 : 0) /2;
            double left = (double) (gamepad1.dpad_left ? 1 : 0) /2;
            double right = (double) (gamepad1.dpad_right ? 1 : 0) /2;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            if (stateMachine == StateMachine.WAITING_FOR_TARGET) {
                frontLeftMotor.setPower(frontLeftPower + up - down - left + right);
                backLeftMotor.setPower(backLeftPower + up - down + left - right);
                frontRightMotor.setPower(frontRightPower + up - down + left - right);
                backRightMotor.setPower(backRightPower + up - down - left + right);
            } else {
                frontLeftMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT));
                frontRightMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT));
                backLeftMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK));
                backRightMotor.setPower(nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
            }
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
            }
            if (gamepad1.a) {
                stateMachine = StateMachine.DRIVE_TO_TARGET_A;
                nav.driveTo(odo.getPosition(), TARGET_A, 0.3, 0);
            } else {
                stateMachine = StateMachine.WAITING_FOR_TARGET;
            }
            coDriver();


        }
    }
    public void coDriver() {
        checkClaw();
        checkBucket();
        checkSlide();
        checkArm();
    }
    public void checkClaw() {
        if (gamepad2.right_trigger > 0.5) {
            servoPinceR.setPower(-1);
            servoPinceL.setPower(1);
        } else if (gamepad2.left_trigger > 0.5) {

            servoPinceR.setPower(1);
            servoPinceL.setPower(-1);

        } else {
            servoPinceR.setPower(0);
            servoPinceL.setPower(0);
        }
        intakeMotor.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
    }
    public void checkBucket() {
        if (gamepad2.right_bumper) {
            servoBucket.setPower(-1);
        } else if (gamepad2.left_bumper) {
            servoBucket.setPower(1);
        } else {
            servoBucket.setPower(0);
        }
    }
    public void checkSlide() {
        if (gamepad2.a) {
            if (!slideMotor.isBusy())
                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideMotor.setTargetPosition(POSITION_1);
            slideMotor.setPower(1.0);

        } else if (gamepad2.y) {
            if (!slideMotor.isBusy()) {
                slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setTargetPosition(POSITION_2);
                slideMotor.setPower(1.0);
            }

        } else if (gamepad2.x) {
            slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideMotor.setPower(gamepad2.right_stick_y);
            if (gamepad2.options) {
                slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
    public void checkArm(){
        if(slideMotor.getTargetPosition() == POSITION_1) {
            armMotor.setPower(gamepad2.left_stick_y);
        }
        else if(gamepad1.left_stick_x == 0.0 && gamepad1.left_stick_y == 0.0 && gamepad1.right_stick_x == 0.0) {
            armMotor.setPower(0.25);
        }
        else{
            armMotor.setPower(0);
        }
    }
}   // end class
