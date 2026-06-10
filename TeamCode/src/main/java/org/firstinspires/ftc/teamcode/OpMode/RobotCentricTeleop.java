package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Config.Arm;
import org.firstinspires.ftc.teamcode.Config.MecanumDrive;

import java.util.List;


@TeleOp(name = "Mecanum -Robot Centric")
public class RobotCentricTeleop extends OpMode{

    private MecanumDrive drive;
    private GamepadEx player1;
    private Arm arm;
    private double speedMultiply = 1;

    public static double liftP = 0.09, liftI = 0.0, liftD = 0.0002, liftF = 0.0009;


    public static int liftSetPoint = 0;

    public static int poleLevel = 0;
    public static int desiredPoleLevel = 0;

    private int oldLiftSetPoint = 0;

    public static double maxPowerConstant = 1.03;
    private static final PIDFController slidePIDF = new PIDFController(liftP,liftI,liftD, liftF);
    public ElapsedTime timer = new ElapsedTime();
    int liftPos = 0;

    DcMotorEx rightLift, leftLift;


    @Override
    public void init(){
        telemetry.addLine("Initializing...");
        telemetry.update();

        player1 = new GamepadEx(gamepad1);

        drive = new MecanumDrive();
        drive.init(hardwareMap);

        arm = new Arm();
        arm.init(hardwareMap);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");



        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        // slidePIDF.setTolerance(12);


        telemetry.addData("lift position", liftPos);
        telemetry.addData("lift set point", liftSetPoint);


        //  telemetry.addData("lift max power (susmarth)", (liftF * liftPos) + maxPowerConstant);


        arm.armIntake();
        telemetry.addLine("Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        timer.reset();
        player1.readButtons();

        if (poleLevel == 0) {
            liftSetPoint = 0;
        } else if (poleLevel == 1) {
            liftSetPoint = 50;
        } else if (poleLevel == 2) {
            liftSetPoint = 250;
        } else if (poleLevel == 3) {
            liftSetPoint = 550;
        } else if (poleLevel == 4) {
            liftSetPoint = 865; //max 875
        }


//        if (gamepad1.left_trigger > 0.2) {
//            speedMultiply = 0.5;
//        } else{
//            speedMultiply = 1;
//        }

        if (gamepad1.left_trigger > 0.2) {
            arm.clawOpen();
        }

        if (gamepad1.right_trigger > 0.2) {
            arm.clawClose();
        }

            if (player1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                arm.clawClose();
                poleLevel = 0;
            } else if (player1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                poleLevel = desiredPoleLevel;
            }

            if (gamepad1.dpad_up) {
                desiredPoleLevel = 1;
            } else if (gamepad1.dpad_right) {
                desiredPoleLevel = 2;
            } else if (gamepad1.dpad_down) {
                desiredPoleLevel = 3;
            } else if (gamepad1.dpad_left) {
                desiredPoleLevel = 4;
            }


            double forward = player1.getLeftY() * speedMultiply;
            double strafe = player1.getLeftX() * speedMultiply;
            double rotate = player1.getRightX() * speedMultiply;


            /** Send inputs to drive class using method created in Mecanum Drive Class */
            drive.drive(forward, strafe, rotate);


            liftPos = rightLift.getCurrentPosition();

            if (oldLiftSetPoint > liftSetPoint) {
                maxPowerConstant = 0.7;
            } else {
                maxPowerConstant = 1;
            }


            slidePIDF.setPIDF(liftP, liftI, liftD, liftF);

            slidePIDF.setSetPoint(liftSetPoint);


            double liftMaxPower = maxPowerConstant;
            double liftPower = Range.clip(slidePIDF.calculate(liftPos, liftSetPoint), -liftMaxPower, liftMaxPower);

            if (liftPos <= 10 && liftSetPoint <= 10) {
                arm.armIntake();
                rightLift.setPower(0);
                leftLift.setPower(0);
            } else {
                arm.armScore();
                rightLift.setPower(liftPower);
                leftLift.setPower(liftPower);
            }


            oldLiftSetPoint = liftPos;


            telemetry.addData("drivebase power", drive.frontLeftMotor.getPower());
            telemetry.addLine("");
            telemetry.addData("lift position", liftPos);
            telemetry.addData("old lift position", oldLiftSetPoint);
            telemetry.addData("lift set point", liftSetPoint);
            telemetry.addData("lift power", liftPower);
            telemetry.addData("desired pole level", desiredPoleLevel);
            telemetry.addLine("");
            telemetry.addData("loop time (ms)", timer.milliseconds());


            telemetry.update();
        }

}



