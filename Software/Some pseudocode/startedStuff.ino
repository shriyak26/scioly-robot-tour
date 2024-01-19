/*
 * Some pseudo code stuff thing
 * Arduino language
*/


//Variables and Constants

//Define pins:
//...

//Define necessary constants
//...
//maybe have a constant speed for turning alone, we
    //would also know then hwo long it takes to turn and have a var for that
//whats the target time
//set status to false

//Define necessary objects (queues and stuff)
//...

void setup()
{
    //Clear cmd queue
    //Clear the parameter queue

    //Intialize pinmode stuff and motors
    //...

    //Commands add here
    /*
    Add START_WAIT command
    Add CALCULATE_SPEED command
    
    Add every other direction fwd, back, turn, etc.

    Add STOP command
    */

}




void loop()
{
    /*
    call the check button to update status

    if status is true cuz button was pressed or cuz reloop
        get first items from command queue and param queue
    */

   /*
    int currentCmd = front of cmdQueue
    int currentParam = front of paramQueue
   */

    //long switch statement yay
    /*
    switch (currentCmd)
    {
        case START_WAIT:
            break lol dont do anything
        case CALCULATE_SPEED:
            calculateSpeed(target time);
            break;
        case fwd:
            moveForward(currentParam)
            break;
        case TURN_LEFT :
            turnLeft(currentParam);
            break;
        case TURN_RIGHT :
            turnRight(currentParam);
            break;
        case MOVE_TIL : //idk if keep?
            moveTil();
            break;
        case BACKWARD :
            moveBackward(currentParam);
            break;
        case TEST :
            tester();
            break;
        case STOP :
            status = false;
            setup(); <- start over
        default :
            status = false;
    }
    */

}

//add other methods here

//add(cmd)
/*
add command to command queue
add arbitrary param to param queue
*/

//add(cmd,param)
/*
add command to command queue
add param to paramqueue
*/

//checkButton()
/*
if the button is pressed, change the status to true
allowing the loop to actually run code
*/

//calculateSpeed(target time)
/*
    using the different commands in the command queue
    calculate how fast you want robot to go
    do we want different speeds for fwd,bkwd, and turns?
    im thinking turns should be its own constant speed

    loop thru commands
        total number of forward and backwards
        total number of 90 degree turns

    time for one forward backward = 
    (target time - number of turns * const time it takes to turn) / number of forward backwards
    speed = distance of on forward backward (.5m? / time for on forward backward
    




    setSpeed(speed we want)
*/

//setSpeed(speed)
/*
set speeds to motors (called by calculate speed usually)
*/

//moveForward(distance)
/*

hehe...uhhh
set how many encoder pulses things to get to distance travelled
1:100, 14 counts per revolution
1400 encoder pulses per wheel revolution
wheel diameter 60mm

    idk if we need to account for acceleration decceleration

move forward lol
    loop constantly calling courseCorrect()
    until done moving forward ig
*/

//moveBackward(distance)
/*
move backward lol
*/

//turnLeft(degrees)
/*
turn left lol
*/

//turnRight(degrees)
/*
turn right lol
*/

//------actual motors being activated methods--------

//turn both motors on forward
//motorForward()
/*
    call left and right motors forward
*/

//turn both motors on backward
//motorBackward()
/*
    call left and right motors backwards
*/

//each separately
//motorRightForward()
/*
    write to right motor via pin to go forward
*/

//motorLeftForward()
/*
    write to left motor via pin to go forward 
*/

//motorRightBackward()
/*
write to right motor via pin to go backward
*/

//motorLeftBackward()
/*
    write to left motor via pin to go backwards 
*/

//turn both motors off
//motorStop()
/*
write to motors to stop
*/

//courseCorrect()
//for as its moving
/*
    if drifting left, make left motor faster in magnitude
    if drifting right, make right motor faster in magnitude
*/