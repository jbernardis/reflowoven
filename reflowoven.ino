
#include <PID_v1.h>
#include <LiquidTWI2.h>
#include <max6675.h>

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5

// ***** PID PARAMETERS *****
#define PID_SAMPLE_TIME 1000

void lcdInit(boolean);
boolean debounce(int);
void displayTemperature(double, int, int);
void displayTime(double);
void message(char *);
void messages(char *, char *);
double getCurrentTemperature();
void setCurrentProfile(int);
void setStage(int, int);
void pip(void);

const int REFLOW_STATE_PREHEAT = 0;
const int REFLOW_STATE_SOAK = 1;
const int REFLOW_STATE_PREREFLOW = 2;
const int REFLOW_STATE_REFLOW = 3;
const int REFLOW_STATE_COOL = 4;
const int REFLOW_STATE_IDLE = 5;
const int REFLOW_STATE_TOO_HOT = 6;

const int REFLOW_STATUS_OFF = 0;
const int REFLOW_STATUS_ON = 1;

class Profile {
private:
	char * name;
	double soakMin;
	long soakStep;
	long soakTotal;
	double soakMax;
	long reflowStep;
	double reflowMax;
	double kp[4];
	double ki[4];
	double kd[4];
	char description[21];
public:
	Profile (char *nm) {
		name = nm;
	}
	void setDescription(char * desc) {
		strcpy(description, desc);
	}
	void setTemps(double smin, double smax, double rfmax) {
		soakMin = smin;
		soakMax = smax;
		reflowMax = rfmax;
	}
	void setTimes(long sstep, long rstep) {
		// calculate soak step times - the time passed is the total soak time
		// however, we need to divide this into chunks.  The number of chunks
		// is calculated as the total temp increase during soak (soakMax-soakMin)
		// divided by the constant SOAK_TEMPERATURE_STEP
		int nsteps = (soakMax - soakMin)/SOAK_TEMPERATURE_STEP;
		soakTotal = sstep;
		soakStep = sstep / nsteps;
		reflowStep = rstep;
	}
	void setPID(int state, double p, double i, double d) {
		kp[state] = p;
		ki[state] = i;
		kd[state] = d;
	}
	void getTemps(double * skmin, double * skmax, double * reflmax) {
		*skmin = soakMin;
		*skmax = soakMax,
		*reflmax = reflowMax;
	}
	void getTimes(long * stotal, long * sstep, long * rstep) {
		*stotal = soakTotal;
		*sstep = soakStep;
		*rstep = reflowStep;
	}
	char * getDescription() {
		return description;
	}
	void getPID(int state, double * p, double * i, double * d) {
		*p = kp[state];
		*i = ki[state];
		*d = kd[state];
	}
	char *getName(void) {
		return name;
	}
};



// ***** LCD MESSAGES *****
const char* lcdMessagesReflowStatus[] = {
	"Pre-heat  ",
	"Soak      ",
	"Pre-reflow",
	"Reflow    ",
	"Cool      ",
	"Ready     ",
	"Wait-hot  "
};

const char* normalMenu = " Select      Start  ";
const char* runMenu =    "             Cancel ";

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = 100;
double ki = 0.0;
double kd = 20;;
unsigned long windowSize;
unsigned long windowStartTime;
double tempSoakMin = 100;
double tempSoakMax = 150;
double tempReflowMax = 220;
long timeSoakStep = 9000;
long timeSoakTotal = 45000;
long timeReflowStep = 10000;
char profileDescription[21];

// ***** timing variables *****
unsigned long reflowStart;
unsigned long stageStart;

// **** array for profiles
const int NPROFS = 3;
Profile * profile[NPROFS];
int currentProfile = 0;

unsigned long nextRefresh;
unsigned long nextRead;
unsigned long timerSoak;

const int finishFreq = 1000;
const int coolFreq = 3000;
const int pipFreq = 5000;

// Reflow oven controller state machine state variable
int reflowState = REFLOW_STATE_IDLE;

// Reflow oven controller status
int reflowStatus = REFLOW_STATUS_OFF;

// Seconds timer
int timerSeconds;

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Specify the Thermocouple interface
int thermoDO = 4;
int thermoCS = 5;
int thermoCLK = 6;
#define NTEMPS 4
double temps[NTEMPS];
int tx = 0;
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

// Specify LCD interface
LiquidTWI2 lcd(0x20);
const int numRows = 4;
const int numCols = 20;

unsigned long messageClearTime = 0;
unsigned long message2ClearTime = 0;
unsigned long flashClearTime = 0;

// define special LCD characters
byte tempChar[8] =
{
    B00100,
    B01010,
    B01010,
    B01010,
    B01010,
    B10001,
    B10001,
    B01110
};

byte clockChar[8] =
{
    B00000,
    B00000,
    B01110,
    B10011,
    B10101,
    B10001,
    B01110,
    B00000
};

const float analogRefVoltage = 5.0;

// ***** pins *****
const int thermPin = 2;
const int selectPin = 3;
const int startPin = 7;
const int heaterPin = 8;
const int buzzerPin = 9;
const int greenLed = 10;
const int redLed = 13;
const int blueLed = 1;

void setup()
{
	profile[0] = new Profile((char *)"Sn-Pb     ");
	profile[0] -> setDescription("Tin/Lead blend      ");
	profile[0] -> setTemps(100.0, 150.0, 220.0);
	profile[0] -> setTimes(90000L, 10000L);
	profile[0] -> setPID(REFLOW_STATE_PREHEAT, 100.0, 0.025,  20.0);
	profile[0] -> setPID(REFLOW_STATE_SOAK,    300.0, 0.05,  250.0);
	profile[0] -> setPID(REFLOW_STATE_PREREFLOW, 300.0, 0.05,  300.0);
	profile[0] -> setPID(REFLOW_STATE_REFLOW, 300.0, 0.05,  300.0);

	profile[1] = new Profile((char *)"Pb free   ");
	profile[1] -> setDescription("Lead free          ");
	profile[1] -> setTemps(120.0, 217.0, 245.0);
	profile[1] -> setTimes(170000L, 50000L);
	profile[1] -> setPID(REFLOW_STATE_PREHEAT, 100.0, 0.025,  20.0);
	profile[1] -> setPID(REFLOW_STATE_SOAK,    300.0, 0.05,  250.0);
	profile[1] -> setPID(REFLOW_STATE_PREREFLOW, 300.0, 0.05,  300.0);
	profile[1] -> setPID(REFLOW_STATE_REFLOW, 300.0, 0.05,  300.0);

	profile[2] = new Profile((char *)"SnBi LT   ");
	profile[2] -> setDescription("Lead free low temp  ");
	profile[2] -> setTemps(90.0, 130.0, 165.0);
	profile[2] -> setTimes(76000L, 20000L);
	profile[2] -> setPID(REFLOW_STATE_PREHEAT, 100.0, 0.025,  20.0);
	profile[2] -> setPID(REFLOW_STATE_SOAK,    300.0, 0.05,  250.0);
	profile[2] -> setPID(REFLOW_STATE_PREREFLOW, 300.0, 0.05,  300.0);
	profile[2] -> setPID(REFLOW_STATE_REFLOW, 300.0, 0.05,  300.0);

	setCurrentProfile(0);
	reflowOvenPID.SetTunings(kp, ki, kd);

	// init the LCD
	lcdInit(true);

	// SSR pin initialization to ensure reflow oven is off
	pinMode(heaterPin, OUTPUT);
	digitalWrite(heaterPin, LOW);

	// make sure the buzzer is silent
	noTone(buzzerPin);

	// buttons
	pinMode(startPin, INPUT);
	digitalWrite(startPin, HIGH);
	digitalRead(startPin);
	pinMode(selectPin, INPUT);
	digitalWrite(selectPin, HIGH);
	digitalRead(selectPin);

	// LEDs
	pinMode(greenLed, OUTPUT);
	digitalWrite(greenLed, LOW);
	pinMode(redLed, OUTPUT);
	digitalWrite(redLed, LOW);
	pinMode(blueLed, OUTPUT);
	digitalWrite(blueLed, LOW);

	// Set window size
	windowSize = 2000;

	// Initialize time keeping variable
	nextRefresh = millis();
	messageClearTime = nextRefresh;

	// Initialize thermocouple reading variable
	nextRead = millis();

	// initialize the thermocouple
	delay(500);
	for (tx=0; tx<NTEMPS; tx++)
		temps[tx] = thermocouple.readCelsius();
	tx = 0;
}

void loop()
{
	// Current time
	unsigned long now;

	now = millis();

	// Time to read thermocouple?
	if (now >= nextRead) {
                nextRead += 1000;
		input = getCurrentTemperature();
	}

	if (now > messageClearTime && messageClearTime != 0) {
		messageClearTime = 0;
		lcd.setCursor(0, 3);
		if (reflowStatus == REFLOW_STATUS_ON) {
			lcd.print((char *) runMenu);
		}
		else {
			lcd.print(normalMenu);
	  }
	}
	if (now > message2ClearTime && message2ClearTime != 0) {
		message2ClearTime = 0;
		lcd.setCursor(0, 2);
		lcd.print((char *) "                    ");
	}
	if (now >= nextRefresh) {
		nextRefresh += 1000;

		// Print current profile and system state
		lcd.setCursor(0, 0);
		lcd.print(profile[currentProfile]->getName());
		lcd.setCursor(10, 0);
		lcd.print(lcdMessagesReflowStatus[reflowState]);

		if (flashClearTime == 0 || now > flashClearTime || reflowStatus == REFLOW_STATUS_ON) {
			flashClearTime = 0;
			// Print the current temperature and target
			lcd.setCursor(0, 1);
			lcd.write((uint8_t) 0);
			lcd.print(" ");

			displayTemperature(input, 5, 1);
			if (reflowStatus == REFLOW_STATUS_ON) {
				lcd.print("   ");
				displayTemperature(setpoint, 5, 1);
				lcd.print("     ");
			}
			else {
				lcd.print("             ");
			}
		}

		lcd.setCursor(0, 2);
		if (reflowStatus == REFLOW_STATUS_ON) {
			lcd.write((uint8_t) 1);
			lcd.print(" ");
			// Print current times
			displayTime(now - reflowStart);
			lcd.print("   ");
			displayTime(now - stageStart);
		}
	}

	// Reflow oven controller state machine
	switch (reflowState) {
	case REFLOW_STATE_IDLE:
		if (input >= TEMPERATURE_ROOM) {
			reflowState = REFLOW_STATE_TOO_HOT;
		}
                else {
                        digitalWrite(greenLed, LOW);
                }
		break;

	case REFLOW_STATE_PREHEAT:
		reflowStatus = REFLOW_STATUS_ON;
	    
		if (input >= tempSoakMin) {
			// Chop soaking period into smaller sub-period
			timerSoak = millis() + timeSoakStep;

			// Set less agressive PID parameters for soaking ramp
			setStage(currentProfile, REFLOW_STATE_SOAK);
			reflowOvenPID.SetTunings(kp, ki, kd);

			// Ramp up to first section of soaking temperature
			setpoint = tempSoakMin + SOAK_TEMPERATURE_STEP;   
		
			// Proceed to soaking state
			reflowState = REFLOW_STATE_SOAK; 
			stageStart = now;
		}
		break;

	case REFLOW_STATE_SOAK:     
		if ((now > timerSoak) && (input >= setpoint)) {
			timerSoak = now + timeSoakStep;
	      
			setpoint += SOAK_TEMPERATURE_STEP;
			if (setpoint > tempSoakMax) {
				// Set agressive PID parameters for reflow ramp
				setStage(currentProfile, REFLOW_STATE_PREREFLOW);
				reflowOvenPID.SetTunings(kp, ki, kd);

				// Ramp up to reflow temperature - 5
				setpoint = tempReflowMax - 5;   

				// Proceed to prereflowing state
				reflowState = REFLOW_STATE_PREREFLOW; 
				stageStart = now;
			}
		}
		break; 

	case REFLOW_STATE_PREREFLOW:
                if (input >= (tempReflowMax-5)) {
		    // Ramp up to full reflow temp
                    setpoint = tempReflowMax;

                    // Proceed to reflow state
                    reflowState = REFLOW_STATE_REFLOW; 
                    stageStart = now;
                }
		break;   

	case REFLOW_STATE_REFLOW:
		if (now > stageStart + timeReflowStep) {
		//if (input >= (tempReflowMax)) {
			// Ramp down to minimum cooling temperature
			setpoint = TEMPERATURE_COOL_MIN;   

			// Proceed to cooling state
			tone(buzzerPin, coolFreq, 500);
			reflowState = REFLOW_STATE_COOL; 
			digitalWrite(redLed, LOW);
			digitalWrite(blueLed, HIGH);
			stageStart = now;
		}
		break;   

	case REFLOW_STATE_COOL:
		if (input <= TEMPERATURE_COOL_MIN) {
			// Turn off reflow process
			reflowStatus = REFLOW_STATUS_OFF;                
			reflowOvenPID.SetMode(MANUAL);

			// Proceed to reflow Completion state
			reflowState = REFLOW_STATE_IDLE; 

			tone(buzzerPin, finishFreq, 1000);
			digitalWrite(blueLed, LOW);
			digitalWrite(greenLed, HIGH);
			message((char *)"Reflow completed");
		}         
		break;    

	case REFLOW_STATE_TOO_HOT:
		if (input < TEMPERATURE_ROOM) {
			reflowState = REFLOW_STATE_IDLE;
		}
		break;
	}    

	if (debounce(selectPin) == LOW && reflowStatus != REFLOW_STATUS_ON) {
		char buf[16];
                
		while (debounce(selectPin) == LOW);
		pip();

		nextRefresh = now;
		if (currentProfile+1 >= NPROFS)
			setCurrentProfile(0);
		else
			setCurrentProfile(currentProfile+1);
                
		lcd.setCursor(0, 2);
		lcd.print((char *) "                    ");
		lcd.setCursor(0, 2);
		lcd.print((char *) "SK ");
		lcd.write((uint8_t) 0);
		lcd.print((char *) ": ");
		lcd.print((char *) dtostrf(tempSoakMin,3,0,buf));
		lcd.print((char *) "-");
		lcd.print((char *) dtostrf(tempSoakMax,3,0,buf));
		lcd.print((char *) " ");
		lcd.write((uint8_t) 1);
		lcd.print((char *) ": ");
		lcd.print((char *) dtostrf(timeSoakTotal/1000,3,0,buf));

		lcd.setCursor(0, 3);
		lcd.print((char *) "                    ");
		lcd.setCursor(0, 3);
		lcd.print((char *) "RF ");
		lcd.write((uint8_t) 0);
		lcd.print((char *) ": ");
		lcd.print((char *) dtostrf(tempReflowMax,3,0,buf));
		lcd.print((char *) "     ");
		lcd.write((uint8_t) 1);
		lcd.print((char *) ": ");
		lcd.print((char *) dtostrf(timeReflowStep/1000,3,0,buf));

		nextRefresh = millis();
		messageClearTime = nextRefresh + 15000;
		message2ClearTime = messageClearTime;

		flashMessage(profileDescription);
	}

	if (debounce(startPin) == LOW) {
		while (debounce(startPin) == LOW);
		pip();
		nextRefresh = now;
                nextRead = now;
		if (reflowStatus == REFLOW_STATUS_ON) {
			// cancel reflow
			reflowStatus = REFLOW_STATUS_OFF;
			reflowOvenPID.SetMode(MANUAL);
			reflowState = REFLOW_STATE_IDLE;
			digitalWrite(redLed, LOW);
			digitalWrite(blueLed, LOW);
			digitalWrite(greenLed, LOW);
			message((char *)"Reflow cancelled");
		}
		else {
			if (reflowState != REFLOW_STATE_TOO_HOT) {
				// Initialize PID control window starting time
				windowStartTime = now;
				reflowStart = now;
				stageStart = now;
				messageClearTime = 1;
				message2ClearTime = 1;
	
				// Ramp up to minimum soaking temperature
				setpoint = tempSoakMin;
	
				// Tell the PID to range between 0 and the full window size
				reflowOvenPID.SetOutputLimits(0, windowSize);
				reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
	
				// Turn the PID on
				reflowOvenPID.SetMode(AUTOMATIC);
	
				// Proceed to preheat stage
				reflowStatus = REFLOW_STATUS_ON;
				reflowState = REFLOW_STATE_PREHEAT;
				setStage(currentProfile, REFLOW_STATE_PREHEAT);
				reflowOvenPID.SetTunings(kp, ki, kd);
        
				digitalWrite(buzzerPin, LOW);
				digitalWrite(greenLed, LOW);
				digitalWrite(redLed, HIGH);
			}
			else {
				message((char *)"Wait for cooling");
			}
		}
	}

	// PID computation and SSR control
	if (reflowStatus == REFLOW_STATUS_ON) {
		reflowOvenPID.Compute();

		if((now - windowStartTime) > windowSize) { 
			// Time to shift the Relay Window
			windowStartTime += windowSize;
		}
		if(output > (now - windowStartTime))
			digitalWrite(heaterPin, HIGH);
		else
			digitalWrite(heaterPin, LOW);   
	}
	// Reflow oven process is off, ensure oven is off
	else {
		digitalWrite(heaterPin, LOW);
	}
}

void pip() {
	 tone(buzzerPin, pipFreq, 50);
}

void lcdInit(boolean init) {
    if (init) {
		lcd.setMCPType(LTI_TYPE_MCP23008); 
        lcd.begin(numCols, numRows);
		lcd.setBacklight(HIGH);
        lcd.createChar(0, tempChar);
        lcd.createChar(1, clockChar);
    }
    lcd.clear();
}

const int debounceDelay = 10;

boolean debounce(int pin)
{
    boolean state;
    boolean previousState;

    previousState = digitalRead(pin);

    for(int counter=0; counter < debounceDelay; counter++) {
        delay(1);

        state = digitalRead(pin); 
        if( state != previousState) {
            counter = 0;
            previousState = state; 
        }
    }

    return state;
}


void displayTemperature(double value, int width, int precision) {
	char buf[16];
	lcd.print(dtostrf(value, width, precision, buf));
}

void displayTime(double tm) {
	int s = tm / 1000; // convert milliseconds to seconds
	int minutes = s/60; // now convert to minutes and seconds
	int sec = s % 60;
	
	if (minutes < 10) 
		lcd.print(" ");
	lcd.print(minutes);

	lcd.print(":");
	if (sec < 10) 
		lcd.print("0");
	lcd.print(sec);
}

void message(char * msg) {
	lcd.setCursor(0, 3);
	lcd.print((char *) "                    ");
	lcd.setCursor(0, 3);
	lcd.print((char *) msg);
        nextRefresh = millis();
        messageClearTime = nextRefresh + 5000;
        message2ClearTime = 1;
}

void messages(char * msg1, char * msg2) {
	lcd.setCursor(0, 2);
	lcd.print((char *) "                    ");
	lcd.setCursor(0, 2);
	lcd.print((char *) msg1);
	lcd.setCursor(0, 3);
	lcd.print((char *) "                    ");
	lcd.setCursor(0, 3);
	lcd.print((char *) msg2);
        nextRefresh = millis();
        messageClearTime = nextRefresh + 5000;
        message2ClearTime = messageClearTime;
}

void flashMessage(char * msg) {
	lcd.setCursor(0, 1);
	lcd.print((char *) "                    ");
	lcd.setCursor(0, 1);
	lcd.print((char *) msg);
	nextRefresh = millis();
	flashClearTime = nextRefresh + 3000;
}

double getCurrentTemperature() {
	temps[tx] = thermocouple.readCelsius();
	tx++;
	if (tx >= NTEMPS)
		tx = 0;

	double tot = 0.0;

	for (int i=0; i<NTEMPS; i++)
		tot = tot + temps[i];

	double t = tot/(double) NTEMPS;

  double nt = t + 10.0 + (t-50.0)/10.0;
	return (nt);
}

void setCurrentProfile(int px) {
	currentProfile = px;
	profile[px] -> getTemps(&tempSoakMin, &tempSoakMax, &tempReflowMax);
	profile[px] -> getTimes(&timeSoakTotal, &timeSoakStep, &timeReflowStep);
	strcpy(profileDescription, profile[px] -> getDescription());
	setStage(px, REFLOW_STATE_PREHEAT);
}

void setStage(int px, int st) {
		profile[px] -> getPID(st, &kp, &ki, &kd);
}
