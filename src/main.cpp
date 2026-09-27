#include <Arduino.h>
#include "Config.h"
#include "MotorControl.h"
#include "Sensors.h"
#include "Display.h"
#include "Interrupts.h"

MotorControl motors;
Sensors sensors;
Display display;

RobotState currentState= PISTA_A;
int tileScore= 0;

//forward
void runPistaA();
void handleMazeTurn();
void checkLackofProgressButton();

void setup(){
    Serial.begin(115200);
    pinMode(BTN_LOP_PIN, INPUT_PULLUP);

    motors.begin();
    display.begin();

    //initallize ToF Xshut sequense, IMu callibration, RGB
    if(!sensors.begin()){
        display.showStatus("Sensor error","check wiring");
        while(true){
            delay(100);
        }
    }
    setupInterrupt();

    display.showStatus("Pista A","Go");
    delay(1000);
}

void loop(){
    checkLackofProgressButton();

    switch(currentState){
        case PISTA_A:
            runPistaA();
            break;
        
        case ROUND_COMPLETE:
            motors.brakeM();
            display.showStatus("FIN", "");
            while(true){
                delay(100);
            }
            break;

        case LACK_OF_PROGRESS:
            motors.brakeM();
            display.showStatus("LACK OF PROGRESS","CLICK TO RESUME");
            delay(500);
            while(digitalRead(BTN_LOP_PIN)==HIGH){
                delay(10);
            }
            gridSteps= 0;
            currentState= PISTA_A;
            display.showStatus("Resuming", "");
            delay(1000);
            break;

            default:
            motors.brakeM();
            break;
    }
}

void runPistaA(){
    //color
    FloorColor floor= sensors.readFloorColor(true);

    if(floor== RED){
        currentState= ROUND_COMPLETE;
        return;
    }
    else if(floor== CYAN || floor== YELLOW || floor== ORANGE|| floor==MAGENTA){
        tileScore++;
        display.showTile(floor);
    }
    //wall
    if(sensors.isWallAhead()){
        motors.brakeM();
        delay(100);
        handleMazeTurn();
        return;
    }

    //elevation and centering
    float pitch= sensors.getPitch();

    if(abs(pitch) > pitch_ramp){
        motors.rampTorque(pitch);
        display.showStatus("OBSTACLE","");
    }else{
        int leftDist  = sensors.getLeftDist();
        int rightDist = sensors.getRightDist();

        if(leftDist<wall_too_close){
            motors.driveSteering(speed_norm + 35, speed_norm - 35);
        }else if(rightDist < wall_too_close){
            motors.driveSteering(speed_norm - 35, speed_norm + 35);
        }else{
            // centered? cruise straight
            motors.driveForward(speed_norm);
        }
    }
}

void handleMazeTurn(){
    int rightDist= sensors.getRightDist();
    int leftDist= sensors.getLeftDist();
    isTurning= true;

    if(rightDist > corridor_open_dist){
        display.showStatus("TURNING RIGHT", "");
        motors.turnIMU(90.0, true, sensors.getIMU());;
    }
    else if(leftDist > corridor_open_dist){
        display.showStatus("TURNING LEFT", "");
        motors.turnIMU(90.0, false, sensors.getIMU());
    }else{
        display.showStatus("DEAD END", "");
        motors.turnIMU(180.0, true, sensors.getIMU());
    }
    isTurning= false;
    delay(100);
}

void checkLackOfProgressButton() {
    if (digitalRead(BTN_LOP_PIN)== LOW && currentState != LACK_OF_PROGRESS) {
        currentState= LACK_OF_PROGRESS;
    }
}
