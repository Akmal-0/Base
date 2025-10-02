#include <Arduino.h>
#include <config.h>
#include <pid.h>
#include <kinematic.h>
// #include <Adafruit_BNO055.h>

void readEncoder0();
void readEncoder1();
void readEncoder2();
void setMotor(int cwPin, int ccwPin, float pwmVal);

extern const int enca[3];
extern const int encb[3];
extern const int cw[3];
extern const int ccw[3];

volatile long pos[3];

PID wheel1(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel2(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel3(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID wheel4(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
//PID dribble(PWM_MIN, PWM_MAX, drib_kp, drib_ki, drib_kd);
//PID launcher_up(0, 180, ESC_UP_KP, ESC_UP_KI, ESC_UP_KD);
//PID launcher_down(0, 180, ESC_DOWN_KP, ESC_DOWN_KI, ESC_DOWN_KD);

Kinematic kinematic(
	Kinematic::ROBOT_2,
	MOTOR_MAX_RPS,
	MAX_RPS_RATIO,
	MOTOR_OPERATING_VOLTAGE,
	MOTOR_POWER_MAX_VOLTAGE,
	WHEEL_DIAMETER,
	ROBOT_DIAMETER);

void setup() {

	Serial.begin(115200);

	for (int i = 0; i < 3; i++)  // Ubah dari 2 ke 3 untuk 3 motor
	{
		pinMode(cw[i], OUTPUT);
		pinMode(ccw[i], OUTPUT);

		analogWriteFrequency(cw[i], PWM_FREQUENCY);
		analogWriteFrequency(ccw[i], PWM_FREQUENCY);

		analogWriteResolution(PWM_BITS);
		analogWrite(cw[i], 0);
		analogWrite(ccw[i], 0);

		pinMode(enca[i], INPUT);
		pinMode(encb[i], INPUT);
	}

	wheel1.ppr_total(COUNTS_PER_REV1);
	wheel2.ppr_total(COUNTS_PER_REV2);
	wheel3.ppr_total(COUNTS_PER_REV3);

	attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder0, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder1, RISING);
	attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder2, RISING);

	pinMode(LED_PIN, OUTPUT);
}

float toDeg(float rad)
{
	return rad * 360 / (11204);
}

void loop() {
	
}

void setMotor(int cwPin, int ccwPin, float pwmVal)
{
	if (pwmVal > 0)
	{
		analogWrite(cwPin, fabs(pwmVal));
		analogWrite(ccwPin, 0);
	}
	else if (pwmVal < 0)
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, fabs(pwmVal));
	}
	else
	{
		analogWrite(cwPin, 0);
		analogWrite(ccwPin, 0);
	}
}

void readEncoder0()
{
	int b = digitalRead(encb[0]);
	if (b > 0)
	{
		pos[0]++;
	}
	else
	{
		pos[0]--;
	}
}

void readEncoder1()
{
	int b = digitalRead(encb[1]);
	if (b > 0)
	{
		pos[1]++;
	}
	else
	{
		pos[1]--;
	}
}

void readEncoder2()
{
	int b = digitalRead(encb[2]);
	if (b > 0)
	{
		pos[2]++;
	}
	else
	{
		pos[2]--;
	}
}