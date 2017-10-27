
#include <stdio.h>
#include <iostream>

#include "RobotConnector.h"

#include "cv.h"
#include "highgui.h"

using namespace std;

#define Create_Comport "COM3"

bool isRecord = true;

int BW_Divider[4] = {1300,950,750,1050};

CreateData	robotData;
RobotConnector	robot;

ofstream	record;

int escaping_spinningCounter = 0;
bool isEscaping = false;
bool isUturning = false;
bool isFindingLine = false;
bool isSettingUp = true;

int main()
{
	
	record.open("../data/robot.txt");

	if( !robot.Connect(Create_Comport) )
	{
		cout << "Error : Can't connect to robot @" << Create_Comport << endl;
		return -1;
	}

	robot.DriveDirect(0, 0);
	cvNamedWindow("Robot");

	int mode = 0;
	cout << "Setup completed\n";

	while(true)
	{
		char c = cvWaitKey(30);
		if( c == 27 ) break;
	
		double vx, vz;
		vx = vz = 0.0;

		bool isBlack[4]; // isBlack[i] will be true if cliff sensor i detect black
		for (int i = 0; i < 4; i++) {
			if (robotData.cliffSignal[i] < BW_Divider[i]) isBlack[i] = true;
			else isBlack[i] = false;
		}

		if (mode == 1) { // If in counter-clockwise mode, swap sensor 1 and 2
			bool tmp = isBlack[1];
			isBlack[1] = isBlack[2];
			isBlack[2] = tmp;
		}

		if (isSettingUp) { // Currently in setup state (stay still until press Q,E button)
			
			// Stay still
			vx = 0;
			vz = 0;
			
			if (c == 'e' || c == 'q') { // If press button
				if (c == 'e') // Clockwise
					mode = 0;
				else if (c == 'q')// Counter-clockwise
					mode = 1;
				isSettingUp = false; // exit setup state
				if (!isBlack[0] && !isBlack[1] && !isBlack[2] && !isBlack[3]) { // If all sensors are white, enter findling line state
					isFindingLine = true;
				}
				else { // using normal walk state
					isFindingLine = false;
				}
			}
			cout << "SETTING UP\t";
			record << "SETTING UP\t";
		}
		else if (escaping_spinningCounter > 0) { // If in escaping - spinning state

			// Turn around
			vx = 0;
			vz = 1;

			escaping_spinningCounter--;
			cout << "ESCAPE-SPIN\t";
			record << "ESCAPE-SPIN\t";
		}
		else if ((robotData.bumper[0] || robotData.bumper[1]) || isEscaping) { // if hitting something or in escaping state
			
			// Drive backward
			vx = -1;
			vz = 0;

			isEscaping = true;
			if (!robotData.bumper[0] && !robotData.bumper[1]) { // If stopped hitting, do a 180 degree turn
				isEscaping = false;
				if(isFindingLine)
					escaping_spinningCounter = 26;
				else {
					isUturning = true;
					mode = (1 - mode);
				}
			}
			cout << "ESCAPE\t";
			record << "ESCAPE\t";
		}
		else if (isFindingLine) { // If in finding line state

			// Drive forward
			vx = 1;
			vz = 0;
			
			if (isBlack[0] && isBlack[1] && isBlack[2] && isBlack[3]) { // If all sensors are black, do a U-Turn to find the line
				isUturning = true;
				isFindingLine = false;
			}
			cout << "FINDING\t";
			record << "FINDING\t";
		}
		else if (isUturning) { // If in U-Turn state

			// Turn around
			vx = 0;
			vz = 0.5;

			if (isBlack[1] && !isBlack[2]) { // If find the line
				// uturn finished
				isUturning = false;
				cout << "FUTURN\t";
				record << "FUTURN\t";
			}
			else {
				cout << "UTURN\t";
				record << "UTURN\t";
			}
		}
		else if (!isBlack[1]) {

			// Turn left
			vz = 0.25;
			vx = 0.75;

			cout << "LEFT\t";
			record << "LEFT\t";
		}
		else if (isBlack[2]) {

			// Turn right
			vz = -0.25;
			vx = 0.75;

			cout << "RIGHT\t";
			record << "RIGHT\t";
		}
		else {

			// Drive forward
			vx = +1;

			cout << "STR\t";
			record << "STR\t";
		}


		switch(c)
		{
		case 'w': vx = +1; break;
		case 's': vx = -1; break;
		case 'a': vz = +1; break;
		case 'd': vz = -1; break;
		case 't': isUturning = true; mode = 1 - mode; break;
		case 'p': isSettingUp = true; break;
		case ' ': vx = vz = 0; break;
		case 'c': robot.Connect(Create_Comport); break;
		}

		double vl = vx - vz;
		double vr = vx + vz;

		int velL = (int)(vl*Create_MaxVel);
		int velR = (int)(vr*Create_MaxVel);

		if (mode == 1) { // If in counter-clockwise mode, swap vl and vr
			int tmp = velL;
			velL = velR;
			velR = tmp;
		}

		int color = (abs(velL)+abs(velR))/4;
		color = (color < 0) ? 0 : (color > 255) ? 255 : color;

		int inten = (robotData.cliffSignal[1] + robotData.cliffSignal[2])/8 - 63;
		inten = (inten < 0) ? 0 : (inten > 255) ? 255 : inten;

		robot.LEDs(velL > 0, velR > 0, color, inten);
		
		if( !robot.DriveDirect(velL, velR) )
			cout << "SetControl Fail" << endl;

		if( !robot.ReadData(robotData) )
			cout << "ReadData Fail" << endl;

		for (int i = 0; i < 4; i++) {
			if (isBlack[i]) {
				cout << "B\t";
				record << "B\t";
			}
			else {
				cout << "W\t";
				record << "W\t";
			}
		}
		cout << (robotData.bumper[0]?'T':'F') << "\t" << (robotData.bumper[1]?'T':'F') << endl;
		record << (robotData.bumper[0] ? 'T' : 'F') << "\t" << (robotData.bumper[1] ? 'T' : 'F') << endl;
	}

	robot.Disconnect();
	record.close();

	return 0;
}