package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class m extends LinearOpMode {
    double x,i;
    double y;
    float a;

    @Override
    public void runOpMode() throws InterruptedException {
        a=6;
        y=9;
        x=a+y;
        x=a-y;
        x=a/y;
        x=a*y;
        y=a+x;
        if(a == 6){
           a=a+6;
        }
        y=a%5;
        if(x==2){
            x=y-2;
        }
        x=0;
        while(x<6){
            x=x+2;
            while(y<=8){
                y=x+9;
            }
        }
        a=0;
        for(i=1;i<=10;i++){
            a++;
        }
        for(y=1; y<7;y++){
            y++;
        }
        if(a==0){
            x=1;
        }
        else{
            y=0;
        }
    }
}