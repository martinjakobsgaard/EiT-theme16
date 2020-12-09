#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include "rs232.h"

#define BUFFER_SIZE 2
#define FRONT '1'
#define BACK '2'
#define STOP '3'

#define COMPORT 16

class GripperControl {
public:
    GripperControl();

    void setMessage(char _command, char _speed);
    void sendMessage();
    void sendMessage(char _command, char _speed);

private:
    int writer;
    unsigned char message[BUFFER_SIZE] = {0};
};

#endif
