#include <OneWire.h>
#include <DallasTemperature.h>

// digital port 0-3 open
// Data wire is plugged into port 2 on the Arduino

#define ONE_WIRE_BUS 2	// data is on port 2 of the arduino
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire); 

int numberOfDevices; // Number of temperature devices found

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

class TempSensor
{
	/*
	class for temp sensor.  Temp sensor uses I2C bus and is a DS18b20 temperature sensor.  This class handles set up, reading, 
	storage of info needed to display results in the cloud.  Polling is not handled by this class.....mostly because Arduino
	limits this class using another class in an external library, specifically the Timer class.
  
	*/
protected:
	#define TEMPERATURE_PRECISION 9	// temp persion 9 bit
	#define TempSampleInterval 1000	// default sampling interval in ms
	byte			Snum;		// number of the sensor device on the I2C buss
	String			Sname;		// name of sensor to use with logs and displays
	String			SIDstring;	// string of ID for sensor when used in cloud based data logging

	boolean			IsOn;		// true if activly taking sensor readings else false
	int				PollInterval;// polling interval

public:
	float	TempC;					// last temperature reading in celcius.	if -100 then not valid
	float	TempF;					// last temperature reading in farenheight.  If-100 then not valid
	DeviceAddress	Saddr;			// I2C address of the sensor

	void	SetPollInterval(int Delay);	//sets the poll interval, changes the poll interval if sensor IsOn=true
	void	TempSensorInit(byte SN);	//used like constructor because constructor syntax was not working ;-(.  passes in device #
	void	TurnOn(boolean TurnOn);		//if true, set isOn true else set isOn false
	boolean Locate(void);				// locates the temp sensor at Saddr, returns true if found else false
	float	ReadC(void);				// read value of sensor and return temperature in degrees C
	float	ReadF(void);				// read value of sensor and return temp in degrees F
	void	printAddress(void);			// prints the address in Saddr 
} TempSens;

/*--------------------------------------------------------------------------------------------------------------------
	methods for Temperature sensor class
  --------------------------------------------------------------------------------------------------------------------*/

//----------------------------------------------------------------------
void	TempSensor::TempSensorInit(byte SN)
{
	// constructor, 0 stuff out, find device address, set precision
	IsOn = false;
	Sname = SIDstring = "";
	TempC = TempF = -100;	//preset to value indicating invalid temp
	PollInterval = TempSampleInterval;	//default polling rate in MS
	Snum = SN;

	if (sensors.getAddress(Saddr, SN))	// Search the I2C bus for address
	{
		//if true, device address in set in Saddr else error
		Serial.print("Found device number= ");	//debug
		Serial.print(Snum);	//debug
		Serial.print(" with address: "); //debug
		//printAddress(tempDeviceAddress);
		for (uint8_t i = 0; i < 8; i++)
		{
			if (Saddr[i] < 16) Serial.print("0");
			Serial.print(Saddr[i], HEX);
		}
		Serial.println();

		Serial.print("Setting resolution to ");
		Serial.println(TEMPERATURE_PRECISION, DEC);	
		sensors.setResolution(Saddr, TEMPERATURE_PRECISION);// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)

		Serial.print("Resolution actually set to: ");
		Serial.print(sensors.getResolution(Saddr), DEC);
		Serial.println();
	}
	else 
	{
		Serial.print("Error, did not find temp sensor address");	//debug
		//add error log entry here
	}
}
//----------------------------------------------------------------------
void	TempSensor::TurnOn(boolean TurnOn)
{
	//if true, set Turn on sampling 
};	
//----------------------------------------------------------------------
float	TempSensor::ReadC(void) 
{// read value of sensor and return temperature in degrees C
	TempC = sensors.getTempC(Saddr);
	return TempC;	
	Serial.print("Temp C= "); Serial.print(TempC);	//debug
};			
//----------------------------------------------------------------------
float	TempSensor::ReadF(void) 
{// read value of sensor and return temperature in degrees F
	TempF = sensors.getTempF(Saddr);
	return TempF;
	Serial.print("Temp F= "); Serial.print(TempF);	//debug
}
//----------------------------------------------------------------------
void	TempSensor::printAddress(void) 	// prints the address of temp sensor
{
	for (uint8_t i = 0; i < 8; i++)
	{
		if (Saddr[i] < 16) Serial.print("0");
		Serial.print(Saddr[i], HEX);
	}
}


void setup(void)
{
	// start serial port
	Serial.begin(9600);
	Serial.println("Dallas Temperature IC Control Library Demo");
	
	sensors.begin();								// Start up the library
	numberOfDevices = sensors.getDeviceCount();		// get count of devices on the wire
		
	Serial.print("Found "); Serial.print(numberOfDevices, DEC);	Serial.println(" temp sensors.");	//debug

	TempSens.TempSensorInit(0);	// initialize device, get address, set precision	

}


// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
	// method 1 - slower
	//Serial.print("Temp C: ");
	//Serial.print(sensors.getTempC(deviceAddress));
	//Serial.print(" Temp F: ");
	//Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

	// method 2 - faster
	float tempC = sensors.getTempC(deviceAddress);
	Serial.print("Temp C: ");
	Serial.print(tempC);
	Serial.print(" Temp F: ");
	Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}


void loop(void)
{
	// request to all devices on the bus
	Serial.print("Requesting temperatures...");
	sensors.requestTemperatures(); // Send the command to get temperatures
	Serial.println("DONE");
	
	if (sensors.getAddress(tempDeviceAddress, 0))
	{
		// Output the device ID
		Serial.print("Temperature for device 0= ");

		// It responds almost immediately. Let's print out the data
		printTemperature(tempDeviceAddress); // Use a simple function to print out the data
	}
	else Serial.println("problem reading sensor address in loop");

	delay(2000);	// wait a few sec
			
}


