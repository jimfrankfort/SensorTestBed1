#include <OneWire.h>
#include <DallasTemperature.h>
#include <Timer.h>

// digital port 0-3 open
// Data wire is plugged into port 2 on the Arduino

#define ONE_WIRE_BUS 2	// data is on port 2 of the arduino
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire); 

int numberOfDevices; // Number of temperature devices found
Timer SensTmr;		// timer object used for sensors

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
	byte			Snum;			// number of the sensor device on the I2C buss
	String			Sname;			// name of sensor to use with logs and displays
	String			SIDstring;		// string of ID for sensor when used in cloud based data logging
	DeviceAddress	Saddr;			// I2C address of the sensor
	boolean			IsOn;			// true if activly taking sensor readings else false
	int				PollInterval;	// polling interval
	int				SensorPollContext;	//variable set by SensTmr and passed by code into SensTmr.  It is an index for the timer object
public:
	float	TempC;					// last temperature reading in celcius.	if -100 then not valid
	float	TempF;					// last temperature reading in farenheight.  If-100 then not valid
	boolean	TempSensReady;			// tells main loop that there is a temperature reading that is ready to be processed


	void	TempSensorInit(byte SN);	//used like constructor because constructor syntax was not working ;-(.  passes in device #
	void	TurnOn(boolean TurnOn);		//turn on/off flags and timers used to take readings at intervals
	void	ReadTempSensor(void);		// called at polling intervals, reads the temp sensor, and sets results and flags indicating a reading is ready for use
	void	SetPollInterval(int Delay);	//sets the poll interval, changes the poll interval if sensor IsOn=true	//boolean Locate(void);				// locates the temp sensor at Saddr, returns true if found else false
	//float	ReadC(void);				// converts 
	//float	ReadF(void);				// read value of sensor and return temp in degrees F
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
	TempC = TempF = -100;	//preset to value indicating invalid temp...probably not needed because of TempSensReady
	PollInterval = TempSampleInterval;	//default polling rate in MS
	Snum = SN;
	TempSensReady = false;	// tells main loop that the temperature readings are not ready to be read

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
	if(TurnOn)
	{
		//if here, then we want to turn on the polling for taking temperature readings.  We use SensTmr object of the Timer class
		IsOn = true;	//flag that we are taking Temperature sensor readings
		SensorPollContext = SensTmr.every(PollInterval, SensorPollRedirect, (void*)2);	// begin polling temp readings, call SensorPollRedirect at intervals of PollInterval. timer index = SensorPollContext
	}
	else
	{ 
		// turn off polling for temperature sensor readings
		IsOn = false;	//flag that we are not taking temperature sensor readings
		SensTmr.stop(SensorPollContext);	//turns off the poll timer for this context
	}
};	
//----------------------------------------------------------------------
void	TempSensor::ReadTempSensor(void) 
{
	// called at polling intervals (SensorPollRedirect is called at intervals set up by TurnOn and calls the routing. See SensorPollRedirect for expalination of need for indirection
	// This routine reads the temp sensor, and sets results and flags indicating a reading is ready for use
	sensors.requestTemperatures(); // Send the command to get temperatures
	if (sensors.requestTemperaturesByAddress(Saddr))
	{
		//if here, read ok so get result
		TempC= sensors.getTempC(Saddr);
		TempF = DallasTemperature::toFahrenheit(TempC);	//convert to farenheit
		TempSensReady = true;	// tells main loop that temp is ready to read
	}
	else
	{
		//error in requesting temp reading
		TempSensReady = false;	// not ready to read as there was a problem
		Serial.println("unexpected error calling sensors.requestTemperaturesByAddress in TempSensor::ReadTempSensor");	//debug
		//errorLog here
	}

}
//----------------------------------------------------------------------
void	TempSensor::SetPollInterval(int Delay)
{
	//saves the poll interval, changes the poll interval if sensor IsOn=true
	PollInterval = Delay;
	if(IsOn)
	{
		SensTmr.stop(SensorPollContext);	//turns off the poll timer for this context		
		SensorPollContext = SensTmr.every(PollInterval, SensorPollRedirect, (void*)2);	// begin polling temp readings at new polling interval
	}
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
//----------------------------------------------------------------------
void SensorPollRedirect(void* context)
{
	// this routine exists outside of the Sensor class because we can't use some timer.every method within a class in the .pde implementation.  Compiler cannot resolve which routine to call.
	TempSens.ReadTempSensor();
}


void setup(void)
{
	// start serial port
	Serial.begin(9600);
	Serial.println("Dallas Temperature IC Control Library Demo");
	
	sensors.begin();								// Start up the library
	numberOfDevices = sensors.getDeviceCount();		// get count of devices on the wire
		
	Serial.print("Found "); Serial.print(numberOfDevices, DEC);	Serial.println(" temp sensors.");	//debug

	TempSens.TempSensorInit(0);		// initialize device, get address, set precision	
	TempSens.SetPollInterval(2000);	// set polling interval to 2 sec
	TempSens.TurnOn(true);			//begin polling
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
	// setup initialized the temp sensor and initiated polling
	if (TempSens.TempSensReady)
	{
		TempSens.TempSensReady = false;	//reset because we are processing this 
		Serial.print("Temperature for device 0 = "); Serial.println(TempSens.TempF);
	}

			
}


