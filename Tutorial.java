package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Tutorial")
public class Tutorial extends LinearOpMode
{
    // Khai báo các biến liên quan đến servo, động cơ HD và Core Hex
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor coreHexMotorLeft = null;
    private DcMotor coreHexMotorRight = null;
    private Servo grippedServo;
    private Servo wristServo;
    private boolean manualMode = false;
    private double armSetPoint = 0.0;

    // Khai báo thêm các vị trí cũng như là position của servo và Core Hex
    private final double armManualDeadband = 0.03;
    private final double gripperClosedPosition = 1.0;
    private final double gripperOpenPosition = 0.5;
    private final double wristUpPosition = 1.0;
    private final double wristDownPosition = 0.0;
    private final int armHomePosition = 0;
    private final int armIntakePosition = 10;
    private final int armScorePosition = 600;
    private final int armShutdownThreshold = 5;

    @Override
    public void runOpMode()
    {
        // Định nghĩa từng biến cho từng bộ phận của robot 
        // Servo
        grippedServo = hardwareMap.get(Servo.class, "servo1");
        wristServo = hardwareMap.get(Servo.class, "servo2");

        // Core Hex
        coreHexMotorLeft = hardwareMap.get(DcMotor.class, "coreHexMotor1");
        coreHexMotorRight = hardwareMap.get(DcMotor.class, "coreHexMotor2");

        // Động cơ HD
        leftBackDrive  = hardwareMap.get(DcMotor.class, "motor1");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motor2");


        // Khai bao hướng đi cũng như là chế độ lái cho core hex 
        coreHexMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        coreHexMotorLeft.setDirection(DcMotor.Direction.REVERSE);

        coreHexMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coreHexMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        coreHexMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHexMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        coreHexMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coreHexMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        coreHexMotorLeft.setPower(0.0);
        coreHexMotorRight.setPower(0.0);


        // Khai bao hướng đi cũng như là chế độ lái cho động cơ HD
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialzed");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {
            transferStrucuture(leftBackDrive, rightBackDrive);
            wristedAndGrippedStructure(coreHexMotorLeft, coreHexMotorRight, wristServo, grippedServo);
        }
    }

    public void transferStrucuture(DcMotor leftBackDrive, DcMotor rightBackDrive)
    {
        leftBackDrive.setPower(gamepad2.left_stick_y);
        rightBackDrive.setPower(gamepad2.right_stick_y);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left: ", "%4.2f", gamepad2.left_stick_y);
        telemetry.addData("Right left: ", "%4.2f", gamepad2.right_stick_y);
        telemetry.update();
    }

    public void wristedAndGrippedStructure(DcMotor coreHexMotorLeft, DcMotor coreHexMotorRight, Servo wristServo, Servo grippedServo)
    {
        // Setup the arm with 2 Core Hex Motor
        double manualArmPower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(manualArmPower) > armManualDeadband)
        {
            if (!manualMode)
            {
                coreHexMotorLeft.setPower(0.0);
                coreHexMotorRight.setPower(0.0);
                coreHexMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                coreHexMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                manualMode = true;
            }
            coreHexMotorLeft.setPower(manualArmPower);
            coreHexMotorRight.setPower(manualArmPower);
        }
        else
        {
            if (manualMode)
            {
                coreHexMotorLeft.setTargetPosition(coreHexMotorLeft.getCurrentPosition());
                coreHexMotorRight.setTargetPosition(coreHexMotorRight.getCurrentPosition());
                coreHexMotorLeft.setPower(1.0);
                coreHexMotorRight.setPower(1.0);
                coreHexMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                coreHexMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                manualMode = false;
            }
            if (gamepad1.a)
            {
                wristServo.setPosition(wristUpPosition);
            }
            else if (gamepad1.b)
            {
                wristServo.setPosition(wristDownPosition);
            }
        }
        
        // Re-zero encoder button
        if (gamepad1.start)
        {
            coreHexMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            coreHexMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            coreHexMotorLeft.setPower(0.0);
            coreHexMotorRight.setPower(0.0);
            manualMode = false;
        }
        
        // Watchdog to shut down motor once the arm reaches the home position
        if (!manualMode && coreHexMotorLeft.getMode() == DcMotor.RunMode.RUN_TO_POSITION &&
                           coreHexMotorLeft.getTargetPosition() <= armShutdownThreshold &&
                           coreHexMotorLeft.getCurrentPosition() <= armShutdownThreshold)
        {
            coreHexMotorLeft.setPower(0.0);
            coreHexMotorRight.setPower(0.0);
            coreHexMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            coreHexMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        
        // Gripped structure
        if (gamepad1.left_bumper || gamepad1.right_bumper)
        {
            grippedServo.setPosition(gripperOpenPosition);
        }
        else
        {
            grippedServo.setPosition(gripperClosedPosition);
        }

        // Show the notification and status of Core Hex Motor
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Manual Power", manualArmPower);
        telemetry.addData("Arm Pos:", "left = " +
                         ((Integer)coreHexMotorLeft.getCurrentPosition()).toString() + ", right = " +
                         ((Integer)coreHexMotorRight.getCurrentPosition()).toString());
        telemetry.addData("Arm Pos:", "left = " +
                         ((Integer)coreHexMotorLeft.getTargetPosition()).toString() + ", right = " +
                         ((Integer)coreHexMotorRight.getTargetPosition()).toString());
        telemetry.update();
    }

}
