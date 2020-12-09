#include "gripper_control.h"

int main()
{
	GripperControl gripperControl;
	gripperControl.sendMessage(FRONT, 125);

	return 0;
}
