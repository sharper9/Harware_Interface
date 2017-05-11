#ifndef DRIVE_TO_DIG_H
#define DRIVE_TO_DIG_H
#include "procedure.h"

class DriveToDig : public Procedure
{
public:
    // Methods
    bool runProc();
    // Members
	float chosenHeading; // deg
};

#endif // DRIVE_TO_DIG_H
