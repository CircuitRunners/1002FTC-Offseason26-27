package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private Servo claw;
    private Servo armLeft;
    private Servo armRight;

    private Servo[] arm;

    public void init(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "claw");
        armLeft = hardwareMap.get(Servo.class, "armL");
        armRight = hardwareMap.get(Servo.class, "armR");
        //servo = hardwareMap.get(Servo.class, "hoodServo");


        armRight.setDirection(Servo.Direction.REVERSE);




        arm = new Servo[]{armLeft,armRight};

    }

    public void armScore(){
        for (Servo servo : arm) {
            servo.setPosition(0.92);
        }
    }
    public void armIntake(){
        for (Servo servo : arm) {
            servo.setPosition(0.08);
        }
    }
    public void clawOpen(){
        claw.setPosition(0.65);
    }
    public void clawClose(){
        claw.setPosition(0.515);
    }

    public double getClawPosition(){
        return claw.getPosition();
    }


}
