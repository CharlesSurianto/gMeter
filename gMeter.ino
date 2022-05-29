#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>

const int H_STRIP_PIN = PB5;      //
const int V_STRIP_PIN = PC14;     //
const int BUZZER_PIN = PB10;      //
const int BUTTON_PIN = PA3;       //
const int LEFT_BUTTON_PIN = PB12; //
const int RIGHT_BUTTON_PIN = PB2; //
const int SENS_ADDRESS = 0;       // EEPROM address of sens
const int PIXELS_PER_STRIP = 10;  //
const int BRIGHTNESS = 20;        // 0-255
const int SLOWINTERVAL = 200;     // ms
const int FASTINTERVAL = 100;     // ms
const int N_SAMPLES = 50;         // average samples
const double MIN_LIMIT = 1.0;     // m/s2
const double MAX_LIMIT = 15.0;    // m/s2
const bool DEBUG_MODE = false;    //

Adafruit_MPU6050 mpu;
Adafruit_NeoPixel hStrip(PIXELS_PER_STRIP, H_STRIP_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel vStrip(PIXELS_PER_STRIP, V_STRIP_PIN, NEO_GRB + NEO_KHZ800);

const uint32_t COLOR[5] = {
    0x00ff00,  // green
    0xaaff00,  // greenish yellow
    0xffaa00,  // orange
    0xff0000,  // red
    0xff0000}; // red

int interval = 0,
    sens = 0,
    currentIndex = 0;
bool timeStarted = 0,
     beepState = 0;
double xOffset = 0,
       yOffset = 0,
       zOffset = 0,
       limit,
       xCalibrated[N_SAMPLES] = {0},
       resultant[N_SAMPLES] = {0},
       xTotal = 0,
       resultantTotal = 0;
unsigned long tBeep = 0,
              startPushTime = 0;

void setup(void)
{
    hStrip.begin();
    vStrip.begin();

    hStrip.setBrightness(BRIGHTNESS);
    vStrip.setBrightness(BRIGHTNESS);

    pinMode(BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(LEFT_BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(RIGHT_BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(BUZZER_PIN, OUTPUT);
    // pinMode(PA0, INPUT_PULLUP);

    // for testing the strip
    // if (!digitalRead(PA0))
    // {
    //     while (!digitalRead(PA0))
    //         ;
    //     int mode = 0;
    //     const uint32_t TEST_COLOR[4] = {
    //         hStrip.Color(255, 255, 255),
    //         hStrip.Color(255, 0, 0),
    //         hStrip.Color(0, 255, 0),
    //         hStrip.Color(0, 0, 255),
    //     };
    //     while (1)
    //     {
    //         if (pushed(RIGHT_BUTTON_PIN, HIGH, 5, true))
    //             mode++;
    //         if (pushed(LEFT_BUTTON_PIN, HIGH, 5, true))
    //             mode--;
    //         if (mode < 0)
    //             mode = 4;
    //         if (mode > 4)
    //             mode = 0;
    //
    //         switch (mode)
    //         {
    //         case 4:
    //             while (!digitalRead(LEFT_BUTTON_PIN) && !digitalRead(RIGHT_BUTTON_PIN))
    //             {
    //                 for (int i = 0; i <= 65535 && !digitalRead(LEFT_BUTTON_PIN) && !digitalRead(RIGHT_BUTTON_PIN); i += 16)
    //                 {
    //                     hStrip.rainbow(i, -1, 255, 255, true);
    //                     hStrip.show();
    //                     vStrip.rainbow(i, -1, 255, 255, true);
    //                     vStrip.show();
    //                 }
    //             }
    //             break;
    //         default:
    //             hStrip.fill(TEST_COLOR[mode]);
    //             vStrip.fill(TEST_COLOR[mode]);
    //             hStrip.show();
    //             vStrip.show();
    //             break;
    //         }
    //
    //         if (pushed(PA0, LOW, 5, true))
    //             break;
    //     }
    // }

    sens = EEPROM.read(SENS_ADDRESS);
    limit = (double)sens * (MAX_LIMIT - MIN_LIMIT) / 20.0 + MIN_LIMIT;

    delay(1000);

    if (DEBUG_MODE)
        Serial.begin(115200);

    if (!mpu.begin())
    {
        if (DEBUG_MODE)
            Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    calibrate(false);
}

void loop(void)
{
    // read from sensors
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // extract accelerations
    double xReading = (double)a.acceleration.x;
    double yReading = (double)a.acceleration.y;
    double zReading = (double)a.acceleration.z;

    // subtract the sample from total
    xTotal -= xCalibrated[currentIndex];
    resultantTotal -= resultant[currentIndex];

    // compute calibrated
    xCalibrated[currentIndex] = xReading - xOffset;

    double yCalibrated = yReading - yOffset;
    double zCalibrated = zReading - zOffset;
    resultant[currentIndex] = sqrt(pow(yCalibrated, 2) + pow(zCalibrated, 2));
    // set the sign
    if (zCalibrated >= 0)
        resultant[currentIndex] *= -1;

    // add the sample
    xTotal += xCalibrated[currentIndex];
    resultantTotal += resultant[currentIndex];

    // compute average
    double xAvg = xTotal / N_SAMPLES;
    double resultantAvg = resultantTotal / N_SAMPLES;

    // compute levels from accelerations
    int xLevel = -1 * rawToLevel(xAvg, limit, 5);
    int yLevel = rawToLevel(resultantAvg, limit, 5);

    // display on x axis
    if (xLevel < 0)
    {
        hStrip.fill(hStrip.Color(0, 0, 0), 5, 5);
        int ledIndex = 0, level = -5;
        for (ledIndex, level; level < xLevel; ledIndex++, level++)
        {
            hStrip.setPixelColor(ledIndex, hStrip.Color(0, 0, 0));
        }
        for (ledIndex, level; level < 0; ledIndex++, level++)
        {
            hStrip.setPixelColor(ledIndex, COLOR[level * -1 - 1]);
        }
    }
    else
    {
        hStrip.fill(hStrip.Color(0, 0, 0), 0, 5);
        int ledIndex = 9, level = 5;
        for (ledIndex, level; level > xLevel; ledIndex--, level--)
        {
            hStrip.setPixelColor(ledIndex, hStrip.Color(0, 0, 0));
        }
        for (ledIndex, level; level > 0; ledIndex--, level--)
        {
            hStrip.setPixelColor(ledIndex, COLOR[level - 1]);
        }
    }

    // display on y axis
    if (yLevel < 0)
    {
        vStrip.fill(vStrip.Color(0, 0, 0), 0, 5);
        int ledIndex = 9, level = -5;
        for (ledIndex, level; level < yLevel; ledIndex--, level++)
        {
            vStrip.setPixelColor(ledIndex, vStrip.Color(0, 0, 0));
        }
        for (ledIndex, level; level < 0; ledIndex--, level++)
        {
            vStrip.setPixelColor(ledIndex, COLOR[level * -1 - 1]);
        }
    }
    else
    {
        vStrip.fill(vStrip.Color(0, 0, 0), 5, 5);
        int ledIndex = 0, level = 5;
        for (ledIndex, level; level > yLevel; ledIndex++, level--)
        {
            vStrip.setPixelColor(ledIndex, vStrip.Color(0, 0, 0));
        }
        for (ledIndex, level; level > 0; ledIndex++, level--)
        {
            vStrip.setPixelColor(ledIndex, COLOR[level - 1]);
        }
    }

    // conditions for buzzer
    if (xLevel == -5 || xLevel == 5 || yLevel == -5 || yLevel == 5)
    {
        interval = FASTINTERVAL;
    }
    else if (xLevel == -4 || xLevel == 4 || yLevel == -4 || yLevel == 4)
    {
        interval = SLOWINTERVAL;
    }
    else
    {
        interval = 0;
    }

    int pushState = (int)digitalRead(LEFT_BUTTON_PIN) + (int)digitalRead(RIGHT_BUTTON_PIN);

    if (pushState > 0 && !timeStarted)
    {
        startPushTime = millis();
        timeStarted = true;
    }
    else if (pushState == 0)
    {
        timeStarted = false;
    }

    if (pushState == 1 && millis() - startPushTime >= 150)
    {
        adjustSens();
        timeStarted = false;
    }
    if (pushState == 2 && millis() - startPushTime >= 150)
    {
        calibrate(true);
        timeStarted = false;
    }

    beep(interval);
    hStrip.show();
    vStrip.show();

    if (DEBUG_MODE)
    {
        Serial.print(xReading);
        Serial.print("\t");
        Serial.print(xCalibrated[currentIndex]);
        Serial.print("\t");
        Serial.print(yReading);
        Serial.print("\t");
        Serial.print(yCalibrated);
        Serial.print("\t");
        Serial.print(zReading);
        Serial.print("\t");
        Serial.print(zCalibrated);
        Serial.print("\t");
        Serial.print(limit);
        Serial.print("\t");
        Serial.print(xAvg);
        Serial.print("\t");
        Serial.print(xLevel);
        Serial.print("\t");
        Serial.print(resultantAvg);
        Serial.print("\t");
        Serial.print(yLevel);
        Serial.print("\t");
        Serial.print(pushState);
        Serial.println();
    }

    currentIndex++;
    if (currentIndex >= N_SAMPLES)
        currentIndex = 0;
}

void beep(unsigned long interval)
{
    if (interval > 0)
    {
        if (millis() - tBeep >= interval)
        {
            if (beepState)
            {
                digitalWrite(BUZZER_PIN, LOW);
            }
            else
            {
                digitalWrite(BUZZER_PIN, HIGH);
            }
            beepState = !beepState;
            tBeep = millis();
        }
    }
    else
    {
        digitalWrite(BUZZER_PIN, LOW);
    }
}

int rawToLevel(double raw, double maxRaw, int level)
{
    int i;
    if (raw < 0)
    {
        for (i = 0; i > -1 * level; i--)
        {
            if (raw > maxRaw / level * i - maxRaw / level)
                return i;
        }
    }
    else
    {
        for (i = 0; i < level; i++)
        {
            if (raw < maxRaw / level * i + maxRaw / level)
                return i;
        }
    }
    return i;
}

void calibrate(bool redFlash)
{
    digitalWrite(BUZZER_PIN, LOW);
    if (DEBUG_MODE)
        Serial.println("calibrate");

    hStrip.clear();
    vStrip.clear();
    hStrip.show();
    vStrip.show();

    // animate wait
    if (redFlash)
        for (int i = 0; i < 5; i++)
        {
            vStrip.setPixelColor(0, vStrip.Color(255, 0, 0));
            vStrip.show();
            delay(200);
            vStrip.clear();
            vStrip.show();
            delay(200);
        }

    xOffset = 0;
    yOffset = 0;
    zOffset = 0;
    int sampleCount;

    for (sampleCount = 0; sampleCount < 200; sampleCount++)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        xOffset += (double)a.acceleration.x;
        yOffset += (double)a.acceleration.y;
        zOffset += (double)a.acceleration.z;

        int progress = sampleCount / 40;
        hStrip.clear();
        vStrip.clear();
        hStrip.setPixelColor(4 - progress, COLOR[progress]);
        hStrip.setPixelColor(5 + progress, COLOR[progress]);
        vStrip.setPixelColor(4 - progress, COLOR[progress]);
        vStrip.setPixelColor(5 + progress, COLOR[progress]);
        hStrip.show();
        vStrip.show();

        delay(1);
    }

    hStrip.clear();
    vStrip.clear();
    hStrip.show();
    vStrip.show();

    xOffset /= sampleCount;
    yOffset /= sampleCount;
    zOffset /= sampleCount;
}

void adjustSens(void)
{
    digitalWrite(BUZZER_PIN, LOW);
    if (DEBUG_MODE)
        Serial.println("adjust sens");

    displaySens();

    unsigned long startTime = millis();
    while (millis() - startTime < 1000)
    {
        if (pushed(LEFT_BUTTON_PIN, HIGH, 5, true))
        {
            sens--;
            if (sens < 0)
                sens = 0;
            startTime = millis();
            displaySens();
            if (DEBUG_MODE)
                Serial.println(sens);
        }
        if (pushed(RIGHT_BUTTON_PIN, HIGH, 5, true))
        {
            sens++;
            if (sens > 20)
                sens = 20;
            startTime = millis();
            displaySens();
            if (DEBUG_MODE)
                Serial.println(sens);
        }
    }
    EEPROM.write(SENS_ADDRESS, sens);
    limit = (double)sens * (MAX_LIMIT - MIN_LIMIT) / 20.0 + MIN_LIMIT;
}

bool pushed(int pin, bool heldState, unsigned long debounce, bool wait)
{
    unsigned long start = millis();
    while (digitalRead(pin) == heldState)
    {
        if (millis() - start > debounce)
        {
            if (wait)
                while (digitalRead(pin) == heldState)
                    ;
            return true;
        }
    }
    return false;
}

void displaySens(void)
{
    hStrip.clear();
    vStrip.clear();

    int left = sens / 4;
    int right = (sens + 2) / 4;
    int up = (sens + 3) / 4;
    int down = (sens + 1) / 4;

    for (int index = 4, level = 0; level < left; index--, level++)
    {
        hStrip.setPixelColor(index, COLOR[level]);
    }
    for (int index = 5, level = 0; level < right; index++, level++)
    {
        hStrip.setPixelColor(index, COLOR[level]);
    }
    for (int index = 4, level = 0; level < up; index--, level++)
    {
        vStrip.setPixelColor(index, COLOR[level]);
    }
    for (int index = 5, level = 0; level < down; index++, level++)
    {
        vStrip.setPixelColor(index, COLOR[level]);
    }

    hStrip.show();
    vStrip.show();
}