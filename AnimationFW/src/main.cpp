#include "Arduino.h"
#include "FastLED.h"

#include "fastledconfig.h"
#include "ledcoords.h"

CRGBArray<NUM_LEDS> leds;

DEFINE_GRADIENT_PALETTE( snow_gp ) {
   0,    0,   0,   0,
  64,    0,   0,   0,
 128,   40, 64, 176,
 192,  255, 255, 128,
 255,  255, 255, 255,
};
CRGBPalette256 snowPalette = snow_gp;

uint32_t tStart = 0;

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);

  delay(3000);

  FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
    .setCorrection(0xFFB08C);

  tStart = millis();

  for(int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
}

// Updates a snowfall-like effect where clouds of Perlin noise fall downwards at random angles.
void drawSnowfall()
{
  static constexpr uint32_t k_windChangeMilliseconds = 30000;
  static constexpr uint32_t k_windTransitionMilliseconds = 5000;

  static constexpr uint16_t k_velRandomMax = 60;
  static constexpr int k_zVel = -130;

  static int offset[3] = {0};

  static int prevVelocity[3] = {0};
  static int velocity[3] = {0};
  static int nextVelocity[3] = {0};

  static uint32_t lastTs = millis();
  static uint32_t lastWindChangeTs = 0;
  static bool bInitialize = true;

  uint32_t ts = millis();

  // Initialize velocities on startup
  if(bInitialize)
  {
    nextVelocity[0] = 0;
    nextVelocity[1] = 0;
    nextVelocity[2] = k_zVel;

    memcpy(prevVelocity, nextVelocity, sizeof(prevVelocity));

    bInitialize = false;
  }

  // Update wind velocity
  if( ts - lastWindChangeTs > k_windChangeMilliseconds )
  {
    lastWindChangeTs = ts;

    // Shift current target velocity to prev velocity
    memcpy(prevVelocity, nextVelocity, sizeof(prevVelocity));

    // Roll a random wind velocity.
    uint16_t rx = random16(0, 2*k_velRandomMax);
    uint16_t ry = random16(0, 2*k_velRandomMax);

    nextVelocity[0] = (int)(rx - k_velRandomMax);
    nextVelocity[1] = (int)(ry - k_velRandomMax);
    nextVelocity[2] = k_zVel;
  }

  // Compute current velocity
  float flVelTransition = ((float)(ts - lastWindChangeTs))/k_windTransitionMilliseconds;
  flVelTransition = fminf(1.0f, flVelTransition);

  velocity[0] = (1.0f - flVelTransition) * prevVelocity[0] + flVelTransition * nextVelocity[0];
  velocity[1] = (1.0f - flVelTransition) * prevVelocity[1] + flVelTransition * nextVelocity[1];
  velocity[2] = (1.0f - flVelTransition) * prevVelocity[2] + flVelTransition * nextVelocity[2];

  // Update position
  uint32_t dt = ts - lastTs;
  offset[0] += velocity[0] * dt;
  offset[1] += velocity[1] * dt;
  offset[2] += velocity[2] * dt; 

  // Sample & draw Perlin noise 
  uint32_t ledPointBuf16p16[3] = {0};
  for(int i = 0; i < NUM_LEDS; i++)
  {
    const uint32_t* const pLed16p16 = LEDPoints::getLED16p16(i);

    if(pLed16p16[0] == 0)
    {
      // untracked LED - turn off
      leds[i] = CRGB(0, 0, 0);
      continue;
    }

    ledPointBuf16p16[0] = pLed16p16[0]*2 + offset[0];
    ledPointBuf16p16[1] = pLed16p16[1]*2 + offset[1];
    ledPointBuf16p16[2] = pLed16p16[2]*2 + offset[2];

    int16_t perlinSample = inoise16(ledPointBuf16p16[0], ledPointBuf16p16[1], ledPointBuf16p16[2]);
    int8_t perlinSample8 = (uint8_t)(perlinSample >> 8);

    // to LED color
    leds[i] = ColorFromPalette(snowPalette, perlinSample8);
  }

  lastTs = ts;
}

void drawPlanes()
{
  constexpr uint32_t tSweep = 3000;
  constexpr uint32_t tCycle = tSweep * 3;

  constexpr float flSweepWidth = 0.05f;
  constexpr float flSweepDarkWidth = flSweepWidth * 3;

  uint32_t ts = millis();

  uint32_t nAxisSweep = (ts % tCycle) / tSweep;
  uint32_t eraseDir = (ts / tSweep) % 2;

  float flSweepPos = (float)(ts % tSweep) / tSweep;
  float flSweepOfs = -0.5f + 1.0f * flSweepPos;

  flSweepOfs = flSweepOfs * 1.5;

  for(int i = 0; i < NUM_LEDS; i++)
  {
    const float* const pLedF = LEDPoints::getLEDFloat(i);

    if(pLedF[0] == NAN)
    {
      continue;
    }

    float flOfs = pLedF[nAxisSweep] - flSweepOfs;
    float flOfsAbs = fabsf(flOfs);

    // Erase ahead or behind
    if(eraseDir == 0 && flOfs < 0 || eraseDir == 1 && flOfs > 0)
      leds[i] = CRGB(leds[i].b, leds[i].g, leds[i].r);

    if( flOfsAbs < flSweepDarkWidth )
    {
      leds[i] = CRGB(0, 0, 0);

      if(flOfsAbs < flSweepWidth)
        leds[i] = CRGB(192, 192, 192);
    }
  }
}

void loop()
{
  drawSnowfall();

  // Sweep planes "erasing" and "showing" the snowfall every so often.
  drawPlanes();

  FastLED.setBrightness(205);
  FastLED.show();
}
