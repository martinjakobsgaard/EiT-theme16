#include "gripper_control.h"

GripperControl::GripperControl()
{
    writer = RS232_OpenComport(COMPORT, 9600, "8N1", 0);
}

void GripperControl::setMessage(char _command, char _speed) {
    message[0] = _command;
    message[1] = _speed;
}

void GripperControl::sendMessage() {
    RS232_SendBuf(COMPORT, message, BUFFER_SIZE);
}

void GripperControl::sendMessage(char _command, char _speed) {
    setMessage(_command, _speed);
    RS232_SendBuf(COMPORT, message, BUFFER_SIZE);
}
