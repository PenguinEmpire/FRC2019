#pragma once

#include <string>
#include "frc/WPILib.h"
#include "PenguinJoystick.h"
#include "AHRS.h"


class Lidar2 {
public:
	Lidar2();
	unsigned int AquireDistance(/*Timer**/);
private:
	enum Address {ADDRESS_DEFAULT=0x62}; // default I2C bus address for the LIDAR-Lite v2
	enum Register {COMMAND=0x00, STATUS=0x01, DISTANCE_1_2=0x8f};
	enum Command {ACQUIRE_DC_CORRECT=0x04};
	enum NumberOfRegistersToRead {READ_1_REGISTER=0x01, READ_2_REGISTERS=0x02};
	enum NumberOfRegistersToWrite {WRITE_1_REGISTER=0x01};
	I2C* I2CBus;

	bool Busy()
	{
		unsigned char Status[Lidar2::READ_1_REGISTER];
		unsigned char statusRegister[Lidar2::WRITE_1_REGISTER];
		statusRegister[Lidar2::WRITE_1_REGISTER-1] = Lidar2::STATUS;

		/**********read status**********/
		if ( I2CBus->WriteBulk(statusRegister, Lidar2::WRITE_1_REGISTER)) {printf ( "WriteBulk status failed! line %d\n", __LINE__ ); return true;}
		if ( I2CBus->ReadOnly(Lidar2::READ_1_REGISTER, Status) ) {printf ( "ReadOnly status failed! line %d\n", __LINE__ ); return true;}
		//printf("Status at line %d %0x, bit0=%0x\n", __LINE__, Status[0], Status[0] & (unsigned char)0x01);
		return (Status[0] & (unsigned char)0x01); // bit 0 is Lidar Lite v2 busy bit
	};
};

Lidar2::Lidar2()
{
	I2CBus = new I2C(I2C::kOnboard, Lidar2::ADDRESS_DEFAULT);
	// Wait(1.);
}

unsigned int Lidar2::AquireDistance(/*Timer* m_timer*/)
{
	unsigned char distance[Lidar2::READ_2_REGISTERS];
	unsigned char distanceRegister_1st[Lidar2::WRITE_1_REGISTER];
	distanceRegister_1st[Lidar2::WRITE_1_REGISTER - 1] = Lidar2::DISTANCE_1_2;

//	printf("Time =  %f starting Lidar2::AquireDistance\n", m_timer->Get());

//	do{Wait(.0001);} while (Busy());

//	printf("Time =  %f acquiring distance\n", m_timer->Get());

	/***********acquire distance**********/		//	WriteBulk() also works
	if ( I2CBus->Write(Lidar2::COMMAND, Lidar2::ACQUIRE_DC_CORRECT) )printf ( "Write operation failed! line %d\n", __LINE__ ); // initiate distance acquisition with DC stabilization

//	do{Wait(.0001);} while (Busy());

//	printf("Time =  %f reading distance\n", m_timer->Get());

	/**********read distance**********/     // Read() does not work
	if ( I2CBus->WriteBulk(distanceRegister_1st, Lidar2::WRITE_1_REGISTER)) printf ( "WriteBulk distance failed! line %d\n", __LINE__ );
	else
	if ( I2CBus->ReadOnly(Lidar2::READ_2_REGISTERS, distance)) printf ( "ReadOnly distance failed! line %d\n", __LINE__ );

	unsigned int dist = (unsigned int)(distance[0]<<8) + (unsigned int)(distance[1]);

//	printf("Time =  %f, Distance= %d (0x%0x)\n", m_timer->Get(), dist, dist);
	return dist;
}