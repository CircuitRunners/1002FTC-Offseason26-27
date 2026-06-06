package org.firstinspires.ftc.teamcode.OpMode.Tuners;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;

import java.util.List;

@Configurable
@TeleOp
public class SlidesTuner extends OpMode {

    //range is 0 - 875
    public static double liftP = 0.09, liftI = 0.0, liftD = 0.0002, liftF = 0.0009;


    public static int liftSetPoint = 0;

    private int oldLiftSetPoint = 0;

    public static double maxPowerConstant = 1.03;
    private static final PIDFController slidePIDF = new PIDFController(liftP,liftI,liftD, liftF);
    public ElapsedTime timer = new ElapsedTime();
    int liftPos = 0;

    DcMotorEx rightLift, leftLift;


    @Override
    public void init() {
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

        telemetry.update();
    }

    @Override
    public void loop() {
        timer.reset();

        liftPos = rightLift.getCurrentPosition();

        if (oldLiftSetPoint > liftSetPoint){
            maxPowerConstant = 0.7;
        }
        else{
            maxPowerConstant = 1;
        }


        slidePIDF.setPIDF(liftP, liftI, liftD, liftF);

        slidePIDF.setSetPoint(liftSetPoint);


        double liftMaxPower = maxPowerConstant;
        double liftPower = Range.clip(slidePIDF.calculate(liftPos, liftSetPoint), -liftMaxPower, liftMaxPower);

//        double pivotMaxPower = maxPowerConstant;

        // liftPower = (liftPower / Math.abs(liftPower)) * Math.sqrt(Math.abs(liftPower));
        rightLift.setPower(liftPower);
        leftLift.setPower(liftPower);

        oldLiftSetPoint = liftPos;

        // pivotPower = (pivotPower / Math.abs(pivotPower)) * Math.sqrt(Math.abs(pivotPower));



        telemetry.addData("lift position", liftPos);
        telemetry.addData("old lift position", oldLiftSetPoint);
        telemetry.addData("lift set point", liftSetPoint);
        telemetry.addData("lift power", liftPower);

        telemetry.addData("loop time (ms)", timer.milliseconds());

        telemetry.update();
    }

}

