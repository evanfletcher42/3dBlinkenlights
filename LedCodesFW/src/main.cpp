#include "Arduino.h"
#include "FastLED.h"

#include "fastledconfig.h"
#include "hadamardcodes.h"

CRGBArray<NUM_LEDS> leds;

void setup()
{
  pinMode(LED_BUILTIN,OUTPUT);

  delay( 3000 ); //safety startup delay

  FastLED.setMaxPowerInVoltsAndMilliamps( VOLTS, MAX_MA);
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
    .setCorrection(0xFFB08C);
    // .setCorrection(0xFFFFFF);
}

void animateLeds()
{
  const uint8_t kIntensitySync = 72;
  const uint8_t kIntensityOn = 36;
  const uint8_t kIntensityOff = 12;

  for(int nCdmaBitIdx = 0; nCdmaBitIdx < 10; nCdmaBitIdx++)
  {
    // Sync on last bit
    if( nCdmaBitIdx >= 8 )
    {
      FastLED.showColor( CRGB(kIntensitySync, kIntensitySync, kIntensitySync) );
    } else
    {
      // Otherwise, blink out the Walsh code for this LED
      for(uint16_t nLedIdx = 0; nLedIdx < NUM_LEDS; nLedIdx++)
      {
        const uint8_t* encodedR;
        const uint8_t* encodedG;
        const uint8_t* encodedB;

        Hadamard::pEncodeRGB(nLedIdx, encodedR, encodedG, encodedB);

        uint8_t red = ( encodedR[ nCdmaBitIdx ] == 1 ? kIntensityOn : kIntensityOff );
        uint8_t grn = ( encodedG[ nCdmaBitIdx ] == 1 ? kIntensityOn : kIntensityOff );
        uint8_t blu = ( encodedB[ nCdmaBitIdx ] == 1 ? kIntensityOn : kIntensityOff );

        leds[nLedIdx] = CRGB(red, grn, blu); 
      }

      FastLED.show();
    }
    delay(66);
  }
}

void loop()
{
  animateLeds();
}
