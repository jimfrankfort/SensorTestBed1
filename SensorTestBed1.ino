#include <OneWire.h>
#include <DallasTemperature.h>
#include <Timer.h>

Timer SensTmr;		// timer object used for sensors

/* variables to monitor duration of main loop.  Can be used to alter*/
long MainLoopStartTime;
long MainLoopEndTime;
long MainLoopDurration;

/*
-------------------------------------------------------------Flow Sensor variablesw and class--------------------------------------------------
YF-S201 Hall Effect Water Flow Meter / Sensor
three flow sensors attached, each to its own input pin.
We use a timer event and polling to read the value of the flow sensors, incrementing a flow sensor
specific counter when the state changes (no debounce needed with Hall effect sensors).
Polling frequency just above 2x expected for max flow rate. In current application read
at 200/sec or once every 5 ms.
Four functions used:
FlowCalcSetup: sets input pins and variables
FlowCalcBegin: initiates the timers
FlowCalcTick: called every 5ms, checks for state changes of flow sensors and increments cntrs
FlowCalcRead: called to read and report flow rates of all sensors

Read Water Flow Meter and output reading in liters/hour
*/
class FlowSensors
{
protected:
	#define	flowmeter1  2		// 1st flow Meter on pin 2
	#define	flowmeter2  3		//2nd flow meter 0n pin3
	#define flowmeter3  4		//3rd flow meter on pin 4
	boolean	usingFlowSensors = false;	// if true, then in the state of using flow sensors, else not
	unsigned long ReadFlowInterval = 1000;		// interval to read flow sensors at
	int8_t flowTickContext;			// index to FlowTick timer in SensTmr
	int8_t flowReadContext;			// index to FlowRead timer in SensTmr
	unsigned long flow1tick = 0;	//frequency counter for flowsensor 1
	unsigned long flow2tick = 0;	//frequency counter for flowsensor 2
	unsigned long flow3tick = 0;	//frequency counter for flowsensor 3
	boolean FlowState1 = false;		// high/low state of input for flow sensor 1
	boolean FlowState2 = false;		// state of flow sensor 2
	boolean FlowState3 = false;		//state of flow sensor 3

	boolean flow1 = false;
	boolean flow2 = false;
	boolean flow3 = false;

	unsigned long flow1dur = 0;		//duty cycle for flowsensor 1
	unsigned long flow2dur = 0;		//duty cycle for flowsensor 2
	unsigned long flow3dur = 0;		//duty cycle for flowsensor 3

	unsigned long flowStartTime;	//starting time for flow calculations
	unsigned long flowEndTime;		//ending time for flow calculationsunsigned 
	long currentTime;
	unsigned long cloopTime;

public:
	unsigned int	FlowValue1;				// flow meter 1 reading in l/min
	unsigned int	FlowValue2;				// reading for meter 2
	unsigned int	FlowValue3;				// reading for meter 3
	boolean FlowReadReady = false;			//used in main loop, if true that ok to read flow values 1-3

	void FlowCalcSetup(void);				//sets up pins for flow sensors	
	void FlowStartStop(boolean Start);		// if Start=true then enable soft interupts to measure flow, else turn them off.
	void SetReadFlowInterval(unsigned long interval);	//sets soft interupt for how often to read the flow sensors
	void FlowCalcTick(void);				// called to check for changes of state of flow sensors

	void FlowCalcBegin(void);				// sets counters at the start of a flow calculation
	void FlowCalcRead(void);				// reads the values of all flow sensors
	
} FlowSens;

	void FlowSensors::FlowCalcSetup()		//sets up pins for flow sensors and sets the timers to measure and read flow sensors
	{
		pinMode(flowmeter1, INPUT);
		pinMode(flowmeter2, INPUT);
		pinMode(flowmeter3, INPUT);
	}
	//----------------------------------------------------------------------
	void FlowSensors::FlowStartStop(boolean Start)		// if Start=true then enable soft interupts to measure flow, else turn them off.
	{
		if (Start)
		{
			flowTickContext = SensTmr.every(5, FlowCalcTickRedirect, (void*)2);	// calls FlowCalcTick every 5ms to check for state change
			flowReadContext = SensTmr.every(ReadFlowInterval, FlowCalcReadRedirect, (void*)3);	// calls to read flow sensors
			usingFlowSensors = true;	// tells us we are using the flow sensore....timers on
			FlowCalcBegin();			// reset counters used for flow calculations
		}
		else 
		{
			FlowReadReady = false;
			usingFlowSensors = false;			// tells us we are not currently using the flow sensors...no timers
			SensTmr.stop(flowTickContext);		// turn off timer for FlowCalcTickRedirect
			SensTmr.stop(flowReadContext);		// turn off timer to read flow sensors
		}
	}
	//----------------------------------------------------------------------
	void FlowSensors::SetReadFlowInterval (unsigned long interval)
	{
		ReadFlowInterval = interval;
		if (usingFlowSensors)
		{
			// we are using the timers for to read the flow sensors, so change the timer interval
			SensTmr.stop(flowReadContext);		// turn off timer to read flow sensors
			flowReadContext = SensTmr.every(ReadFlowInterval, FlowCalcReadRedirect, (void*)3);	// start timer up again with the new interval
			FlowCalcBegin();	//reset counters to insure correct flow readings on the next read cycle
		}
	}
	//----------------------------------------------------------------------
	void FlowSensors::FlowCalcBegin()		// sets counters at the start of a flow calculation
	{
		flow1tick = flow2tick = flow3tick = 0;	// zero out flow sensor frequency counters
		flowStartTime = millis();				 // set starting time for measurement interval. note, millis wraps every ~ 50 days so need to check if end <start when reading
	}
	//----------------------------------------------------------------------
	void FlowSensors::FlowCalcTick(void)			// called by FlowcalcTickRedirect to check for changes of state of flow sensors
	{
		//flow1=digitalRead(flowmeter1);
		//flow2=digitalRead(flowmeter2);
		//flow3=digitalRead(flowmeter3);

		if (digitalRead(flowmeter1) != FlowState1)
		{
			FlowState1 = !FlowState1;	//invert flow state, begin checking for next state change
			flow1tick++;				//increment frequency counter because pin changed state
		}

		if (digitalRead(flowmeter2) != FlowState2)
		{
			FlowState2 = !FlowState2;	//invert flow state, begin checking for next state change
			flow2tick++;				//increment frequency counter because pin changed state
		}

		if (digitalRead(flowmeter3) != FlowState3)
		{
			FlowState3 = !FlowState3;	//invert flow state, begin checking for next state change
			flow3tick++;				//increment frequency counter because pin changed state
		}
	}
	//----------------------------------------------------------------------
	void FlowSensors::FlowCalcRead(void)			// reads the values of all flow sensors
	{	// called by timer ever ReadFlowInterval
		long ElapsedTime;	//local variable for elapsed time since last calculation in milliseconds

		flowEndTime = millis();	//ending time for this flow calculation.

		if (flowEndTime>flowStartTime)
		{
			ElapsedTime = flowEndTime - flowStartTime;
			FlowValue1 = (flow1tick * 60 * 1000 / 7.5);// (Pulse frequency x 60 min) * (1000 ms/ElapsedTime in ms) / 7.5Q = flow rate in L/hour
			FlowValue2 = (flow2tick * 60 * 1000 / ElapsedTime / 7.5);
			FlowValue3 = (flow3tick * 60 * 1000 / 7.5);
			FlowReadReady = true;	// set flag indicating flow results are ready to read

									//look at duration of wave form duration because of Nyquist, sampling rate=5ms so avg duration should be > 10ms
			flow1dur = ElapsedTime / (flow1tick + 1);
			flow2dur = ElapsedTime / (flow2tick + 1);
			flow3dur = ElapsedTime / (flow3tick + 1);
		}
		else
		{
			// millisecond timer has wrapped around (~ every 70 days), so can't use this reading
			FlowReadReady = false;
		}

		FlowCalcBegin();	// restart the flow calculation using the current run of millis
	}
	//----------------------------------------------------------------------
	void FlowCalcTickRedirect(void* context)
	{
		// this routine exists outside of the FlowSensor class because we can't use some timer.every method within a class in the .pde implementation.  Compiler cannot resolve which routine to call.
		FlowSens.FlowCalcTick();	// check for changes of state of flow sensors
	}
	//----------------------------------------------------------------------
	void FlowCalcReadRedirect(void* context)
	{
		// this routine exists outside of the FlowSensor class because we can't use some timer.every method within a class in the .pde implementation.  Compiler cannot resolve which routine to call.
		FlowSens.FlowCalcRead();		// call method that reads the flow sensors and sets the flag in the main loop telling us it is time to do something with the results
	}
	//----------------------------------------------------------------------

//-------------------------------------Dallas Temperature Sensor variables and class------------------------------
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

	FlowSens.FlowCalcSetup();		// initialize pins and counters
	FlowSens.SetReadFlowInterval(2000);	// read every 2 seconds
	FlowSens.FlowStartStop(true);		// begin readings
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


