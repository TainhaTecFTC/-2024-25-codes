package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous
public class Autonomo2 extends LinearOpMode {
    double p = 0.3;
    IMU imu;
    IMU.Parameters myIMUparameters;


    DcMotor left;
    DcMotor right;
    private double ticksRotation = 28*10.4329;
    private int diameter = 9;
    double botHeading;
    //botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    
    
    
    public void runOpMode(){
        imu = hardwareMap.get(IMU.class, "imu");
        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(myIMUparameters);
        imu.resetYaw();
        left = hardwareMap.get(DcMotor.class, "motor_esq");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right = hardwareMap.get(DcMotor.class, "motor_dir");


        //Para andar para frente e trás usa o false
        //Para girar para esquerda e Direira usa o true
        
        RunCm(30, 0.2, false); //RunCm (distancia, potencia, modo)
        RunCm(-45, 0.2, true); //RunCm (distancia, potencia, modo)
        RunCm(63, 0.3, false); //RunCm (distancia, potencia, modo)
        RunCm(-70, 0.3, false); //RunCm (distancia, potencia, modo)
        RunCm(25, 0.3, true); //RunCm (distancia, potencia, modo)
        RunCm(28 ,0.3, false); //RunCm (distancia, potencia, modo)
        RunCm(-23, 0.5, true); //RunCm (distancia, potencia, modo)
    }
    public void RunCm(double cm, double power, boolean inverted){
        right.setTargetPosition(ConvertCentimeterToTick(cm));
        left.setTargetPosition(ConvertCentimeterToTick((inverted?-cm:cm)));

        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        left.setPower(power);
        right.setPower(power);
        
        while (right.isBusy() || left.isBusy()){
        }
        
        right.setPower(0.0);
        left.setPower(0.0);
        
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int ConvertCentimeterToTick(double destinyCentimeters) {
        return (int) (ticksRotation * destinyCentimeters) / (int) (diameter * Math.PI);
    }
}
