/*
 * Some pseudo code stuff thing
 * Arduino language
*/


//Variables and Constants

//Define pins:
//...

//Define necessary constants
//...
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
        case fwd:
            moveForward(currentParam)
            break
        case TURN_LEFT :
            turnLeft(currentParam);
            break;
        case TURN_RIGHT :
            turnRight(currentParam);
            break;
        case MOVE_TIL :
            moveTil(currentParam);
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
add arbitrary param queue
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

//setSpeed(speed)
/*
set speeds to motors
*/

//moveForward(distance)
/*
move forward lol
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
//motorFroward()
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

//moveTill(distance)
/*
    hehe...uhhh
    set how many encoder pulses things to get to distance travelled
    1:100, 14 counts per revolution
    1400 encoder pulses per wheel revolution
    wheel diameter 60mm

    idk if we need to account for acceleration decceleration
*/

//tester()
/*
    using the fancy sensor new stuff gyroscope whatever
    figure out position and stuff
*/