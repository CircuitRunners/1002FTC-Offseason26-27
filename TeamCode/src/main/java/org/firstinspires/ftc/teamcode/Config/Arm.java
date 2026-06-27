package org.firstinspires.ftc.teamcode.Config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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


       //armRight.setDirection(Servo.Direction.REVERSE);

        armLeft.setDirection(Servo.Direction.REVERSE);

        claw.setDirection(Servo.Direction.REVERSE);




        arm = new Servo[]{armLeft,armRight};

    }

    public void setArmPos(double pos){
        armLeft.setPosition(Range.scale(pos,0,1,0.1,0.9));
        armRight.setPosition(pos);
    }

    public double getArmLeftPos(){
        return Range.scale(armLeft.getPosition(),0.1,0.9,0,1);
    }

    public double getArmRightPos(){
        return armRight.getPosition();
    }
    public void armScore(){
        for (Servo servo : arm) {
            servo.setPosition(0.25);
        }
    }
    public void armIntake(){
        for (Servo servo : arm) {
            servo.setPosition(1);
        }
    }
    public void clawOpen(){
        claw.setPosition(0.65);
    }
    public void clawClose(){
        claw.setPosition(0.47);
    }

    public double getClawPosition(){
        return claw.getPosition();
    }


}
