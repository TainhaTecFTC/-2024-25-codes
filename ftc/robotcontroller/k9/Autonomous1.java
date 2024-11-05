// package org.firstinspires.ftc.robotcontroller.k9;

// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.Blinker;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;

// @Autonomous(name = "Azul-auto-sexta")
// public class Autonomous1 extends LinearOpMode{
    
//     private DcMotor left;
//     private DcMotor right;
    
//     private int leftpos;
//     private int rightpos;
    
    
    
//     @Override
//     public void runOpMode(){
//         left = hardwareMap.get(DcMotor.class, "motor_esq");
//         right = hardwareMap.get(DcMotor.class, "motor_dir");
        
//         left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//         right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
//         right.setDirection(DcMotorSimple.Direction.REVERSE);

//         leftPos = 0;
//         rightPos = 0;
        
//         waitForStart();
        
//         drive(1000);
//         drive(1000);

        
//     }
    
    
//     private void drive(int leftTarget, int rightTarget, double speed){
    
//     leftsPos += leftTarget;
//     rightPos += rightTarget;
    
//     left.setTargetPosition(leftPos);
//     right.setTargetPosition(rightPos);
    
//     left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
//     left.setPower(speed);
//     right.setPower(speed);


//     while(opModeIsActive() && left.isBusy() && right.isBusy()){
//         idle();
//     }
    
// }

// }



//     // todo: write your code here

package org.firstinspires.ftc.robotcontroller.k9;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Azul-auto-sexta")
public class Autonomous1 extends LinearOpMode {
    
    private DcMotor left;
    private DcMotor right;
    
    private int leftPos;
    private int rightPos;
    private static final int ENCODER_COUNTS_PER_ROTATION = 1120;
    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "motor_esq");
        right = hardwareMap.get(DcMotor.class, "motor_dir");
        
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPos = 0;
        rightPos = 0;
        
        waitForStart();
        
        // drive(-0, 10, 0.5); 
        // drive(-0, -10, 0.5); 
        
        
        // Move forward 3 rotations
        drive(ENCODER_COUNTS_PER_ROTATION * 0, ENCODER_COUNTS_PER_ROTATION * 2, 0.5); 
        
        // Turn left by moving the left motor backward and the right motor forward
        drive(-ENCODER_COUNTS_PER_ROTATION / 2, ENCODER_COUNTS_PER_ROTATION / 2, 0.5);
        
        stopMotors();
        
        
    }
    
    private void drive(int leftTarget, int rightTarget, double speed) {
        leftPos += leftTarget;
        rightPos += rightTarget;
        
        left.setTargetPosition(leftPos);
        right.setTargetPosition(rightPos);
        
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        left.setPower(speed);
        right.setPower(speed);

        while (opModeIsActive() && left.isBusy() && right.isBusy()) {
            idle();
        }
        
        // Stop the motors after reaching the target position
        left.setPower(0);
        right.setPower(0);
        
        // Reset the motor modes
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    
       private void stopMotors() {
        left.setPower(0);
        right.setPower(0);
    }
}
