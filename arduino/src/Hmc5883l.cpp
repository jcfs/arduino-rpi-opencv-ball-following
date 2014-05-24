/*
Hmc5883l.cpp - Class file for the Hmc5883l Triple Axis Magnetometer Arduino Library.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)/ 2012 bildr.org (Arduino 1.0 compatible)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 WARNING: THE Hmc5883l IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for Hmc5883l:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Hmc5883l_3-Axis_Digital_Compass_IC.pdf

*/

#include <Arduino.h> 
#include "Hmc5883l.h"

Hmc5883l::Hmc5883l()
{
  SetScale(1.3);
  SetMeasurementMode(Measurement_Continuous);
}

MagnetometerRaw Hmc5883l::ReadRawAxis()
{
  uint8_t* buffer = Read(DataRegisterBegin, 6);
  MagnetometerRaw raw = MagnetometerRaw();
  raw.XAxis = (buffer[0] << 8) | buffer[1];
  raw.ZAxis = (buffer[2] << 8) | buffer[3];
  raw.YAxis = (buffer[4] << 8) | buffer[5];
  return raw;
}

MagnetometerScaled Hmc5883l::ReadScaledAxis()
{
  MagnetometerRaw raw = ReadRawAxis();
  MagnetometerScaled scaled = MagnetometerScaled();
  scaled.XAxis = raw.XAxis * m_Scale;
  scaled.ZAxis = raw.ZAxis * m_Scale;
  scaled.YAxis = raw.YAxis * m_Scale;
  return scaled;
}

int Hmc5883l::SetScale(float gauss)
{
	uint8_t regValue = 0x00;
	if(gauss == 0.88)
	{
		regValue = 0x00;
		m_Scale = 0.73;
	}
	else if(gauss == 1.3)
	{
		regValue = 0x01;
		m_Scale = 0.92;
	}
	else if(gauss == 1.9)
	{
		regValue = 0x02;
		m_Scale = 1.22;
	}
	else if(gauss == 2.5)
	{
		regValue = 0x03;
		m_Scale = 1.52;
	}
	else if(gauss == 4.0)
	{
		regValue = 0x04;
		m_Scale = 2.27;
	}
	else if(gauss == 4.7)
	{
		regValue = 0x05;
		m_Scale = 2.56;
	}
	else if(gauss == 5.6)
	{
		regValue = 0x06;
		m_Scale = 3.03;
	}
	else if(gauss == 8.1)
	{
		regValue = 0x07;
		m_Scale = 4.35;
	}
	else
		return -1;

	// Setting is in the top 3 bits of the register.
	regValue = regValue << 5;
	Write(ConfigurationRegisterB, regValue);
    Write(0, 6 << 2);
}

int Hmc5883l::SetMeasurementMode(uint8_t mode)
{
	Write(ModeRegister, mode);
}

void Hmc5883l::Write(int address, int data)
{
  Wire.beginTransmission(Hmc5883l_Address);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t* Hmc5883l::Read(int address, int length)
{
  Wire.beginTransmission(Hmc5883l_Address);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.beginTransmission(Hmc5883l_Address);
  Wire.requestFrom(Hmc5883l_Address, length);

  uint8_t buffer[length];
  if(Wire.available() == length)
  {
	  for(uint8_t i = 0; i < length; i++)
	  {
		  buffer[i] = Wire.read();
	  }
  }
  Wire.endTransmission();

  return buffer;
}

float Hmc5883l::get_heading() {
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = ReadScaledAxis();
 
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
    scaled.XAxis -= 223;
    scaled.YAxis -= -26;  
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
    
  // Convert radians to degrees for readability.
  return (float)(heading * 180/M_PI); 
};

