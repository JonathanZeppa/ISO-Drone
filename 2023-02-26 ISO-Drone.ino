// The ISO-Drone - by Jonathan Zeppa
//
// This project was built for the Teensy 4.1 using the audio board.
//
// You will have to provide your own soundfonts.
// The soundfonts that I've used for this project can be found here: https://schristiancollins.com/generaluser.php
// The soundfont extraction tool can be found here: https://github.com/manicken/SoundFontDecoder
// The slider library can be found here: https://github.com/KrisKasprzak/ILI9341_t3_controls

#include <ILI9341_t3.h>           // fast display driver lib
#include <ILI9341_t3_Controls.h>  // custom control define file
#include <XPT2046_Touchscreen.h>
#include <Colors.h>
#include <Bounce2.h>
#include <RotaryEncoder.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// These files are not included, but the specific ones I used:
#include "RockOrgan_samples.h"
#include "SweepPad_samples.h"
#include "Strings_2Pan_samples.h"
#include "StereoGrand_samples.h"
#include "BrassSection_samples.h"
#include "Polysynth_1_samples.h"

//Position of where each band starts
#define BAND1 20
#define BAND2 60
#define BAND3 100
#define BAND4 140
#define BAND5 180
#define BAND6 220
#define BAND7 260
#define BAND8 300
#define BAND9 214

#define SLIDECOLOR 0x6B6D
#define HANDLECOLOR PURPLE
#define BACKCOLOR DARKGREY
//#define LANE 0x294A
#define LANE BLACK

#define TEXTCOLOR C_WHITE
#define MINDB -12
#define MAXDB 12
#define TICK 2
#define SNAP 1

// note your HEIGHT / (max scale) should be an int
// otherwise you may get some draw errors
#define TOP 45
#define HEIGHT 80

int freqControl = 110;
int rezControl = 0;
int sizeControl = 90; // Room Size
int dampControl = 90; // Dampening
int dryControl = 90; // Dry Ouput
int wetControl = 90; // Wet Ouput
int LFOspeed = 0; // speed
int LFOdepth = 127; // depth

// create some band variables
float Band1 = 26; // Detune
int Band2 = 1; // Octave Case -1
float Band3 = 110;// Freq
float Band4 = 0;  // Rez
float Band5 = 90; // Roomsize
float Band6 = 90; // Damp
float Band7 = 90; // Dry
float Band8 = 90; // Wet
float Band9 = 0;  // Speed - LFO
float Band10 = 127; // Depth - LFO

#define TFT_DC 10
#define TFT_CS 9
#define T_CS    4
#define T_IRQ   3

unsigned long i;
int BtnX, BtnY;

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC); //Display object
XPT2046_Touchscreen Touch = XPT2046_Touchscreen(4, 3);
TS_Point TouchPoint;

// create the slider objects
SliderV sBand1(&tft);
SliderV sBand2(&tft);
SliderV sBand3(&tft);
SliderV sBand4(&tft);
SliderV sBand5(&tft);
SliderV sBand6(&tft);
SliderV sBand7(&tft);
SliderV sBand8(&tft);
SliderH sBand9(&tft);
SliderH sBand10(&tft);

// Color definitions in RGB565 format or 5-6-5
const uint16_t BLACK = 0x0000;
const uint16_t BLUE = 0x001F;
const uint16_t RED = 0xFA20;
const uint16_t GREEN = 0x47E0;
const uint16_t CYAN = 0x07FF;
const uint16_t MAGENTA = 0xF81F;
const uint16_t PURPLE = 0x6011;
const uint16_t YELLOW = 0xFD60;
const uint16_t WHITE = 0xFFFF;
//const uint16_t DARKGREY = 0x39C7;
const uint16_t DARKGREY = 0x2104;
const uint16_t LIGHTGREY = 0x6B6D;
const uint16_t PURERED = 0xF800;
const uint16_t MAROON = 0xBAEB;
const uint16_t DARKGREEN = 0x03A0;

AudioSynthWavetable      wavetable1;     //xy=375.75,844.75
AudioSynthWavetable      wavetable2;     //xy=376.75,884.75
AudioSynthWaveform       lfo;      //xy=394.75,1108.75
AudioSynthWaveformDc     lfoenvelope;            //xy=403.75,1068.75
AudioMixer4              mixer3;         //xy=600.7500190734863,1124.750015258789
AudioMixer4              mixer1;         //xy=611.75,842.75
AudioFilterStateVariable filter1;        //xy=839.75,836.75
AudioFilterStateVariable filter2;        //xy=839.75,910.75
AudioEffectFreeverb      freeverb1;      //xy=1011.75,897.75
AudioMixer4              mixer2;         //xy=1181.75,843.75
AudioOutputI2S           i2s1;           //xy=1649.7500228881836,845.7500133514404
AudioConnection          patchCord1(wavetable1, 0, mixer1, 0);
AudioConnection          patchCord2(wavetable2, 0, mixer1, 1);
AudioConnection          patchCord3(lfo, 0, mixer3, 1);
AudioConnection          patchCord4(lfoenvelope, 0, mixer3, 0);
AudioConnection          patchCord5(mixer3, 0, filter2, 1);
AudioConnection          patchCord6(mixer3, 0, filter1, 1);
AudioConnection          patchCord7(mixer1, 0, filter1, 0);
AudioConnection          patchCord8(mixer1, 0, filter2, 0);
AudioConnection          patchCord9(filter1, 0, mixer2, 0);
AudioConnection          patchCord10(filter2, 0, freeverb1, 0);
AudioConnection          patchCord11(freeverb1, 0, mixer2, 1);
AudioConnection          patchCord12(mixer2, 0, i2s1, 0);
AudioConnection          patchCord13(mixer2, 0, i2s1, 1);
AudioControlSGTL5000     sgtl5000_1;     //xy=613.75,963.75

#define r01STEPS 1
#define r01MIN 12
#define r01MAX 71

#define r02STEPS 1
#define r02MIN 12
#define r02MAX 71

#define r03STEPS 1
#define r03MIN 12
#define r03MAX 71

#define r04STEPS 1
#define r04MIN 12
#define r04MAX 71


String Key1[] = {
  "C -1", "C#-1", "D -1", "D#-1", "E -1", "F -1", "F#-1", "G -1", "G#-1", "A -1", "A#-1", "B -1",
  "C 0", "C#0", "D 0", "D#0", "E 0", "F 0", "F#0", "G 0", "G#0", "A 0", "A#0", "B 0",
  "C 1", "C#1", "D 1", "D#1", "E 1", "F 1", "F#1", "G 1", "G#1", "A 1", "A#1", "B 1",
  "C 2", "C#2", "D 2", "D#2", "E 2", "F 2", "F#2", "G 2", "G#2", "A 2", "A#2", "B 2",
  "C 3", "C#3", "D 3", "D#3", "E 3", "F 3", "F#3", "G 3", "G#3", "A 3", "A#3", "B 3",
  "C 4", "C#4", "D 4", "D#4", "E 4", "F 4", "F#4", "G 4", "G#4", "A 4", "A#4", "B 4",
  "C 5", "C#5", "D 5", "D#5", "E 5", "F 5", "F#5", "G 5", "G#5", "A 5", "A#5", "B 5",
  "C 6", "C#6", "D 6", "D#6", "E 6", "F 6", "F#6", "G 6", "G#6", "A 6", "A#6", "B 6",
  "C 7", "C#7", "D 7", "D#7", "E 7", "F 7", "F#7", "G 7", "G#7", "A 7", "A#7", "B 7",
  "C 8", "C#8", "D 8", "D#8", "E 8", "F 8", "F#8", "G 8", "G#8", "A 8", "A#8", "B 8",
  "C 9", "C#9", "D 9", "D#9", "E 9", "F 9"};

const float freq[128] = {
  //    C        C#        D        D#1         E         F        F#         G        G#      A        A#        B  
     8.176,    8.662,    9.177,    9.723,   10.301,   10.913,   11.562,    12.25,   12.978, 13.75,   14.568,   15.434, // -1
    16.352,   17.324,   18.354,   19.445,   20.602,   21.827,   23.125,     24.5,   25.957,  27.5,   29.135,   30.868, // 0
  //   24        25        26        27        28        29        30        31        32      33       34        35
    32.703,   34.648,   36.708,   38.891,   41.203,   43.654,   46.249,   48.999,   51.913,    55,    58.27,   61.735, // 1
    65.406,   69.296,   73.416,   77.782,   82.407,   87.307,   92.499,   97.999,  103.826,   110,  116.541,  123.471, // 2
   130.813,  138.591,  146.832,  155.563,  164.814,  174.614,  184.997,  195.998,  207.652,   220,  233.082,  246.942, // 3
   261.626,  277.183,  293.665,  311.127,  329.628,  349.228,  369.994,  391.995,  415.305,   440,  466.164,  493.883, // 4
   523.251,  554.365,   587.33,  622.254,  659.255,  698.456,  739.989,  783.991,  830.609,   880,  932.328,  987.767, // 5 
  1046.502, 1108.731, 1174.659, 1244.508,  1318.51, 1396.913, 1479.978, 1567.982, 1661.219,  1760, 1864.655, 1975.533, // 6
  2093.005, 2217.461, 2349.318, 2489.016,  2637.02, 2793.826, 2959.955, 3135.963, 3322.438,  3520,  3729.31, 3951.066, // 7
  4186.009, 4434.922, 4698.636, 4978.032, 5274.041, 5587.652, 5919.911, 6271.927, 6644.875,  7040,  7458.62, 7902.133, // 8
  8372.018, 8869.844, 9397.273, 9956.063, 10548.08,  11175.3, 11839.82, 12543.85};

  String octType[] = {"-2  ", "-1  ", "-5th", "-3rd", " 0  ", "+3rd", "+5th", "+1  ",  "+2  "};

int r01Data = 40;
int r02Data = 45;
int r03Data = 40;
int r04Data = 45;
float transpose1 = (freq[r01Data]);// Left  Foot Switch E-2
float transpose2 = (freq[r02Data]);// Right Foot Switch A-2
float transpose3 = (freq[r03Data]);// Left  Foot Switch E-2
float transpose4 = (freq[r04Data]);// Right Foot Switch A-2

int octave = -12;
int octCase = 1;
const float DIV127 = (1.0 / 127.0);
float detuneFactor = 1;
int detuneValue = 0;
float dryVolume = 0.5;
float wetVolume = 0.5;
int preset = 0;
float sizeSet = 0.7;
float dampSet = 0.7;

int FILfreq =  10000;
float FILfactor = 1;

// Setup a RotaryEncoder with 4 steps per latch for the 2 signal input pins:
RotaryEncoder r01(24, 25, RotaryEncoder::LatchMode::FOUR3);
int r01lastPos = -1;
RotaryEncoder r02(26, 27, RotaryEncoder::LatchMode::FOUR3);
int r02lastPos = -1;
RotaryEncoder r03(33, 34, RotaryEncoder::LatchMode::FOUR3);
int r03lastPos = -1;
RotaryEncoder r04(35, 36, RotaryEncoder::LatchMode::FOUR3);
int r04lastPos = -1;

#define B01_PIN 28
int b01 = 1; // Note starts as OFF
Bounce2::Button b01Toggle = Bounce2::Button();
#define B02_PIN 29
int b02 = 1; // Note starts as OFF
Bounce2::Button b02Toggle = Bounce2::Button();
#define B03_PIN 30
int b03 = 1; // Activate Preset Decrement
Bounce2::Button b03Toggle = Bounce2::Button();
#define B04_PIN 31
int b04 = 1; // Activate Preset Increment
Bounce2::Button b04Toggle = Bounce2::Button();
#define B05_PIN 37
int b05 = 1; // C Foot Switch
Bounce2::Button b05Toggle = Bounce2::Button();
#define B06_PIN 38
int b06 = 1; // D Foot Switch
Bounce2::Button b06Toggle = Bounce2::Button();
#define B07_PIN 32
int b07 = 1; // 2-button / 4-button Switcher
Bounce2::Button b07momentary = Bounce2::Button();


void noteOffA() {
  wavetable1.stop();
  wavetable2.stop(); 
  if (b07 == 1) {
  tft.fillTriangle(3, 181, 42, 191, 80, 181, BLACK);
     }
    else if (b07 == 0){     
      tft.fillTriangle(51, 190, 75, 178, 75, 202, BLACK);
    }
  }


void noteOffB() {
  wavetable1.stop();
  wavetable2.stop(); 
  if (b07 == 1) {
    tft.fillTriangle(239, 181, 278, 191, 316, 181, BLACK);
    }
  else if (b07 == 0){
    tft.fillTriangle(286, 190, 310, 178, 310, 202, BLACK); //B
    }
  }

void noteOffC() {
  wavetable1.stop();
  wavetable2.stop(); 
  tft.fillTriangle(51, 222, 75, 210, 75, 234, BLACK); // C
  }

void noteOffD() {
  wavetable1.stop();
  wavetable2.stop(); 
  tft.fillTriangle(286, 222, 310, 210, 310, 234, BLACK);     // D
  }  

void noteOnA() {
  b02 = 1;
  b05 = 1;
  b06 = 1;
  wavetable1.playFrequency(transpose1);
  wavetable2.playFrequency((freq[r01Data + octave]) * detuneFactor);
 
    //Triangle:    -x  A-y  B-x  B-y  C-x  C-y  Color
  if (b07 == 1) {
    tft.fillTriangle(3, 181, 42, 191, 80, 181, WHITE);
    tft.fillTriangle(239, 181, 278, 191, 316, 181, BLACK);}
  if (b07 == 0) {
    tft.fillTriangle(51, 190, 75, 178, 75, 202, WHITE);
    tft.fillTriangle(51, 222, 75, 210, 75, 234, BLACK); // C

    tft.fillTriangle(286, 190, 310, 178, 310, 202, BLACK); // B
    tft.fillTriangle(286, 222, 310, 210, 310, 234, BLACK); // D
    }   
    Serial.println("A");
  }

void noteOnB() {
  b01 = 1;
  b05 = 1;
  b06 = 1;
  wavetable1.playFrequency(transpose2);
  wavetable2.playFrequency((freq[r02Data + octave]) * detuneFactor);
 
  if (b07 == 1) {
  tft.fillTriangle(239, 181, 278, 191, 316, 181, WHITE);
  tft.fillTriangle(3, 181, 42, 191, 80, 181, BLACK);
  }
  if (b07 == 0) {
  tft.fillTriangle(51, 190, 75, 178, 75, 202, BLACK); // A
  tft.fillTriangle(51, 222, 75, 210, 75, 234, BLACK); // C
  tft.fillTriangle(286, 190, 310, 178, 310, 202, WHITE); // B
  tft.fillTriangle(286, 222, 310, 210, 310, 234, BLACK); // D
  }
  Serial.println("B");}

void noteOnC() { //b05
  b01 = 1;
  b02 = 1;
  b06 = 1;
  wavetable1.playFrequency(transpose3);
  wavetable2.playFrequency((freq[r03Data + octave]) * detuneFactor);
 
    //Triangle:    -x  A-y  B-x  B-y  C-x  C-y  Color
  tft.fillTriangle(51, 190, 75, 178, 75, 202, BLACK); // A
  tft.fillTriangle(51, 222, 75, 210, 75, 234, WHITE); // C
  tft.fillTriangle(286, 190, 310, 178, 310, 202, BLACK); // B
  tft.fillTriangle(286, 222, 310, 210, 310, 234, BLACK); // D
  Serial.println("C");}

void noteOnD() { //b06
  b01 = 1;
  b02 = 1;
  b05 = 1;
  wavetable1.playFrequency(transpose4);
  wavetable2.playFrequency((freq[r04Data + octave]) * detuneFactor);

  tft.fillTriangle(51, 190, 75, 178, 75, 202, BLACK); // A
  tft.fillTriangle(51, 222, 75, 210, 75, 234, BLACK); // C
  tft.fillTriangle(286, 190, 310, 178, 310, 202, BLACK); // B
  tft.fillTriangle(286, 222, 310, 210, 310, 234, WHITE); // D
  Serial.println("D");}

// Index to keep track of current preset
int presetNum[] = {0, 1, 2, 3, 4, 5, 6, 7};
int presetIndex = 0;
int arraySize = sizeof(presetNum) / sizeof(presetNum[0]);
int lastSwitchState_1 = digitalRead(B07_PIN);

void presetMenu()
  {
   switch(preset) {
    case 0:
    wavetable1.setInstrument(SweepPad);
    wavetable2.setInstrument(SweepPad);
    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(2);
    tft.setCursor(3, 3 );
    tft.print(F("01:SweepPad      "));
    detuneValue = 26;
    octCase = 1;
    freqControl = 110;
    rezControl = 0;
    sizeControl = 90; // Room Size
    dampControl = 90; // Dampening
    dryControl = 90; // Dry Ouput
    wetControl = 90; // Wet Ouput
    LFOspeed = 0; // speed
    LFOdepth = 127; // depth
    break;

    case 1:
    wavetable2.setInstrument(StereoGrand);
    wavetable1.setInstrument(SweepPad); 
    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(2);
    tft.setCursor(3, 3 );
    tft.print(F("02:Piano/Synth   "));
    detuneValue = 16;
    octCase = 1;
    freqControl = 127;
    rezControl = 1;
    sizeControl = 91; // Room Size
    dampControl = 91; // Dampening
    dryControl = 91; // Dry Ouput
    wetControl = 91; // Wet Ouput
    LFOspeed = 1; // speed
    LFOdepth = 119; // depth
    break;

    case 2:
    wavetable1.setInstrument(RockOrgan);
    wavetable2.setInstrument(RockOrgan);
    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(2);
    tft.setCursor(3, 3 );
    tft.print(F("03:RockOrgan     "));
    detuneValue = 4;
    octCase = 8;
    freqControl = 127;
    rezControl = 0;
    sizeControl = 90; // Room Size
    dampControl = 90; // Dampening
    dryControl = 90; // Dry Ouput
    wetControl = 90; // Wet Ouput
    LFOspeed = 0; // speed
    LFOdepth = 127; // depth
    break;

    case 3:
    wavetable1.setInstrument(SweepPad);
    wavetable2.setInstrument(SweepPad);
    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(2);
    tft.setCursor(3, 3 );
    tft.print(F("04:Dark Drone    "));   
    detuneValue = 33;
    octCase = 1;
    freqControl = 10;
    rezControl = 20;
    sizeControl = 120; // Room Size
    dampControl = 122; // Dampening
    dryControl = 90; // Dry Ouput
    wetControl = 122; // Wet Ouput
    LFOspeed = 10; // speed
    LFOdepth = 127; // depth
    break;

    case 4:
    wavetable1.setInstrument(Strings_2Pan);
    wavetable2.setInstrument(Strings_2Pan);
    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(2);
    tft.setCursor(3, 3 );
    tft.print(F("05:Strings       "));  
    detuneValue = 18;
    octCase = 1;
    freqControl = 124;
    rezControl = 0;
    sizeControl = 93; // Room Size
    dampControl = 93; // Dampening
    dryControl = 93; // Dry Ouput
    wetControl = 93; // Wet Ouput
    LFOspeed = 3; // speed
    LFOdepth = 117; // depth
    break;

    
    case 5:
    wavetable1.setInstrument(BrassSection);
    wavetable2.setInstrument(BrassSection);
    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(2);
    tft.setCursor(3, 3 );
    tft.print(F("06:BrassSection  "));
    
    detuneValue = 19;
    octCase = 1;
    freqControl = 120;
    rezControl = 80;
    sizeControl = 80; // Room Size
    dampControl = 80; // Dampening
    dryControl = 80; // Dry Ouput
    wetControl = 80; // Wet Ouput
    LFOspeed = 127; // speed
    LFOdepth = 107; // depth
    break;

    case 6:
    wavetable2.setInstrument(StereoGrand);
    wavetable1.setInstrument(Strings_2Pan); 
    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(2);
    tft.setCursor(3, 3 );
    tft.print(F("07:Piano/Strings "));
    detuneValue = 16;
    octCase = 1;
    freqControl = 30;
    rezControl = 1;
    sizeControl = 91; // Room Size
    dampControl = 91; // Dampening
    dryControl = 91; // Dry Ouput
    wetControl = 110; // Wet Ouput
    LFOspeed = 10; // speed
    LFOdepth = 127; // depth
    break;


    case 7:
    wavetable2.setInstrument(Polysynth_1);
    wavetable1.setInstrument(Polysynth_1); 
    tft.setTextColor(GREEN, BLACK);
    tft.setTextSize(2);
    tft.setCursor(3, 3 );
    tft.print(F("08:Polysynth_1   "));
  //tft.print(F("00:Name          "));
    detuneValue = 16;
    octCase = 1;
    freqControl = 30;
    rezControl = 1;
    sizeControl = 91; // Room Size
    dampControl = 91; // Dampening
    dryControl = 91; // Dry Ouput
    wetControl = 110; // Wet Ouput
    LFOspeed = 10; // speed
    LFOdepth = 127; // depth
    break;
  }
 
  BandUpdate();
  } 

void BandUpdate() {
  tft.setTextColor(WHITE, BACKCOLOR);
  tft.setTextSize(1);
  sBand1.draw(detuneValue);
  tft.fillRect(14, 28, 18, 10, BACKCOLOR);  // detune
  tft.setCursor(14 , 30 ); tft.print(detuneValue);
  detuneFactor = 1 - (0.05 * (detuneValue * DIV127));
  Serial.println(detuneValue);

  Band2 = octCase;
  octSwitch();
  sBand2.draw(Band2);
  tft.fillRect(55, 28, 18, 10, BACKCOLOR);  // octave
  tft.setCursor(55, 30 ); 
  tft.print(octType[octCase]);

  sBand3.draw(freqControl);
  tft.fillRect(95, 28, 18, 10, BACKCOLOR);
  tft.setCursor(95 , 30 ); tft.print(freqControl); 
  filter1.frequency(10000 * (freqControl * DIV127)); 
  filter2.frequency(10000 * (freqControl * DIV127)); 

  sBand4.draw(rezControl);
  tft.fillRect(135, 28, 18, 10, BACKCOLOR);
  tft.setCursor(135 , 30 ); tft.print(rezControl);
  filter1.resonance((4.3 * (rezControl * DIV127)) + 0.7);
  filter2.resonance((4.3 * (rezControl * DIV127)) + 0.7);

  sBand5.draw(sizeControl);
  tft.fillRect(BAND5, 28, 18, 10, BACKCOLOR);  
  sizeSet = (sizeControl / 127);
  tft.setCursor(BAND5 , 30 ); tft.print(sizeControl);

  sBand6.draw(dampControl);
  tft.fillRect(BAND6, 28, 18, 10, BACKCOLOR); 
  dampSet = (dampControl / 127);
  freeverb1.damping(dampSet);
  tft.setCursor(BAND6 , 30 ); tft.print(dampControl);

  sBand7.draw(dryControl);
  tft.fillRect(BAND7, 28, 18, 10, BACKCOLOR); 
  tft.setCursor(BAND7 , 30 ); tft.print(dryControl);
  dryVolume = (0.7 * (dryControl * DIV127));
  mixer2.gain(0, dryVolume);

  sBand8.draw(wetControl);
  tft.fillRect(294, 28, 18, 10, BACKCOLOR);           
  tft.setCursor(294 , 30 ); tft.print(wetControl);
  wetVolume = (0.7 * (wetControl * DIV127));
  mixer2.gain(1, wetVolume);

  sBand9.draw(LFOspeed);
  tft.fillRect(BAND9, 190, 18, 10, BACKCOLOR); // Speed - LFO
  tft.setCursor(BAND9 , 190 ); tft.print(LFOspeed); 
  lfo.frequency(3 * (LFOspeed * DIV127)); 

  sBand10.draw(LFOdepth);
  tft.fillRect(BAND9, 210, 18, 10, BACKCOLOR);// Depth - LFO
  tft.setCursor(BAND9 , 212 ); tft.print(LFOdepth);
  lfo.amplitude(0.99 * (LFOdepth * DIV127));

} 

void octSwitch () {
 switch (octCase) {
        case 0:
          octave = -24;
          break;
        case 1:
          octave = -12;
          break;
        case 2:
          octave = -7;
          break;    
        case 3:
          octave = -5;
          break;
        case 4:
          octave = 0;
          break;
        case 5:
          octave = 5;
          break;
        case 6:
          octave = 7;
          break;
        case 7:
          octave = 12;
          break;
        case 8:
          octave = 24;
          break;
      }
}

void menuCheck() {

  b07 = digitalRead(B07_PIN);
  if (b07 != lastSwitchState_1) {
      if (b07 == 0) {
      Serial.println("0");
      FourButtonMenu();
      }
      else if (b07 == 1) {
      Serial.println("1");
      TwoButtonMenu();
      }
    }
  lastSwitchState_1 = b07;
  }

void TwoButtonMenu() {
      wavetable1.stop();
      wavetable2.stop();
      tft.setTextSize(4); tft.setTextColor(WHITE, DARKGREY); 
      tft.fillRect(0, 174, 81, 64, BLACK);    // Clear Box LEFT
      tft.fillRect(236, 174, 81, 64, BLACK);  // Clear Box RIGHT
      tft.drawRoundRect(2, 200, 79, 38, 3, 0x6B6D);
      tft.fillRoundRect(3, 201, 77, 36, 3, DARKGREY);
      tft.setCursor(8, 205);  tft.println(Key1[r01Data]);

      tft.drawRoundRect(238, 200, 79, 38, 3, 0x6B6D);
      tft.fillRoundRect(239, 201, 77, 36, 3, DARKGREY);
      tft.setCursor(244, 205); tft.println(Key1[r02Data]);
    }

void FourButtonMenu() {
      wavetable1.stop();
      wavetable2.stop(); 
      tft.fillRect(0, 174, 81, 64, BLACK);    // Clear Box LEFT
      tft.fillRect(237, 174, 80, 64, BLACK);  // Clear Box RIGHT
      //                x   y   w   h    r   color
      tft.drawRoundRect(2, 176, 46, 28, 3, 0x6B6D);
      tft.fillRoundRect(3, 177, 44, 26, 3, DARKGREY);

      tft.drawRoundRect(2, 208, 46, 28, 3, 0x6B6D);
      tft.fillRoundRect(3, 209, 44, 26, 3, DARKGREY);

      tft.drawRoundRect(238, 176, 46, 28, 3, 0x6B6D);
      tft.fillRoundRect(239, 177, 44, 26, 3, DARKGREY);

      tft.drawRoundRect(238, 208, 46, 28, 3, 0x6B6D);
      tft.fillRoundRect(239, 209, 44, 26, 3, DARKGREY);

      tft.setTextSize(2); tft.setTextColor(WHITE, DARKGREY);
      tft.setCursor(8, 183);   tft.println(Key1[r01Data]);
      tft.setCursor(244, 183); tft.println(Key1[r02Data]);
      tft.setCursor(8, 215);   tft.println(Key1[r03Data]);
      tft.setCursor(244, 215); tft.println(Key1[r04Data]);
  }

void setup() {
  tft.setTextSize(1);
  Serial.begin(115200);

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(BLACK);

  // Voice Control Box
  //                x   y  w    h   r   color
  tft.drawRoundRect(2, 22, 79, 150, 3, 0x6B6D);
  tft.fillRoundRect(3, 23, 77, 148, 3, DARKGREY);

  //  Filter Box
  tft.drawRoundRect(83, 22, 79, 150, 3, 0x6B6D);
  tft.fillRoundRect(84, 23, 77, 147, 3, DARKGREY);  

  // LFO Box
  tft.drawRoundRect(83, 174, 153, 64, 3, 0x6B6D);
  tft.fillRoundRect(84, 175, 151, 62, 3, DARKGREY);

  // Verb Box
  tft.drawRoundRect(164, 22, 153, 150, 3, 0x6B6D);
  tft.fillRoundRect(165, 23, 151, 148, 3, DARKGREY);  

  // Slider Lanes
  tft.fillRoundRect(15, 40, 11, 90, 3, LANE);  // 1
  tft.fillRoundRect(55, 40, 11, 90, 3, LANE);  // 2
  tft.fillRoundRect(95, 40, 11, 90, 3, LANE);  // 3 Freq
  tft.fillRoundRect(135, 40, 11, 90, 3, LANE); // 4 Rez
  tft.fillRoundRect(175, 40, 11, 90, 3, LANE); // 5
  tft.fillRoundRect(215, 40, 11, 90, 3, LANE); // 6
  tft.fillRoundRect(255, 40, 11, 90, 3, LANE); // 7
  tft.fillRoundRect(295, 40, 11, 90, 3, LANE); // 8
  tft.fillRoundRect(85, 189, 125, 11, 3, LANE); // 9 Speed
  tft.fillRoundRect(85, 213, 125, 11, 3, LANE); // 10 Depth

  Touch.begin();
  Touch.setRotation(1);
  
  AudioMemory(80);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.8);
  sgtl5000_1.adcHighPassFilterDisable();
  sgtl5000_1.lineInLevel(0,0);  //level can be 0 to 15.  5 is the Teensy Audio Library's default

  // Wavetable mixer
  mixer1.gain(0, 0.5);
  mixer1.gain(1, 0.5);
  mixer1.gain(2, 0.5);
  mixer1.gain(3, 0.5);

  //Main Mixer
  mixer2.gain(0, 0.5);
  mixer2.gain(1, 0.5);

  // LFO Mixer
  mixer3.gain(0, 0.3);
  mixer3.gain(1, 0.3);

  // begin(level, frequency, waveform);
  lfo.begin(0.3,1,WAVEFORM_SINE);
  lfoenvelope.amplitude(0.3);

  filter1.frequency(FILfreq);
  filter2.frequency(FILfreq);  

  freeverb1.roomsize(sizeSet);
  freeverb1.damping(dampSet);
  
  // initialize the sliders
  sBand1.init(BAND1, TOP, 80, 0, 127, 0, 0, SLIDECOLOR, LANE, HANDLECOLOR); // Detune
  sBand2.init(BAND2, TOP, 80, 0, 8, 1, 1, SLIDECOLOR, LANE, HANDLECOLOR);   // Octave
  sBand3.init(BAND3, TOP, 80, 0, 127, 0, 0, SLIDECOLOR, LANE, HANDLECOLOR); // Filter Frequency
  sBand4.init(140, 45, 80, 0, 127, 0, 0, SLIDECOLOR, LANE, HANDLECOLOR);    // Filter Resonance
  sBand5.init(BAND5, TOP, 80, 0, 127, 0, 0, SLIDECOLOR, LANE, HANDLECOLOR); // Roomsize
  sBand6.init(BAND6, TOP, 80, 0, 127, 0, 0, SLIDECOLOR, LANE, HANDLECOLOR); // Damp
  sBand7.init(BAND7, TOP, 80, 0, 127, 0, 0, SLIDECOLOR, LANE, HANDLECOLOR); // Dry
  sBand8.init(BAND8, TOP, 80, 0, 127, 0, 0, SLIDECOLOR, LANE, HANDLECOLOR); // Wet
  sBand9.init(90, 194, 110, 0, 127, 0, 0, SLIDECOLOR, LANE, HANDLECOLOR);   // Speed - LFO
  sBand10.init(90, 218, 110, 0, 127, 0, 0, SLIDECOLOR, LANE, HANDLECOLOR);  // Depth - LFO

  // draw the slider controls with initial settings
  sBand1.draw(Band1);
  sBand2.draw(Band2);
  sBand3.draw(Band3);
  sBand4.draw(Band4);
  sBand5.draw(Band5);
  sBand6.draw(Band6);
  sBand7.draw(Band7);
  sBand8.draw(Band8);
  sBand9.draw(Band9);
  sBand10.draw(Band10);

  presetMenu();
  wavetable1.amplitude(0.8);
  wavetable2.amplitude(0.8);

  tft.setTextColor(GREEN, BLACK);
  tft.setTextSize(2);
  tft.setCursor(3, 3 );
  tft.print(F("01:SweepPad      "));
 
  tft.setTextColor(WHITE, DARKGREY);
  tft.setTextSize(1);
  tft.setCursor(14, 152);  tft.print(F("V O I C E"));
  tft.setCursor(90, 152);  tft.print(F("F I L T E R"));
  tft.setCursor(208, 152); tft.print(F("R E V E R B"));
  tft.setCursor(144, 227); tft.print(F("L F O"));

  // This where the intial data is printed
  tft.setCursor(14 , 30 ); tft.print(Band1, 0);
  tft.setCursor(55 , 30 ); tft.print(octType[Band2]); // Initial Octave Name
  tft.setCursor(95 , 30 ); tft.print(Band3, 0);
  tft.setCursor(135 , 30 ); tft.print(Band4, 0);
  tft.setCursor(BAND5 , 30 ); tft.print(Band5, 0);
  tft.setCursor(BAND6 , 30 ); tft.print(Band6, 0);
  tft.setCursor(BAND7 , 30 ); tft.print(Band7, 0);
  tft.setCursor(294 , 30 ); tft.print(Band8, 0);
  tft.setCursor(BAND9 , 190  ); tft.print(Band9, 0); // Speed 
  tft.setCursor(BAND9 , 212 ); tft.print(Band10, 0); // Depth

  // Sets the bottom text, both title and position
  tft.setCursor(5 , 132); tft.print(F("Detune"));
  tft.setCursor(44 , 132); tft.print(F("Octave"));
  tft.setCursor(88, 132); tft.print(F("Freq")); 
  tft.setCursor(130, 132); tft.print(F("Rez"));

  tft.setCursor(169 , 132); tft.print(F("Room"));
  tft.setCursor(169 , 140); tft.print(F("Size"));

  tft.setCursor(210, 132); tft.print(F("Damp"));
  tft.setCursor(250, 132); tft.print(F("Dry"));
  tft.setCursor(290, 132); tft.print(F("Wet"));

  tft.setCursor(90, 178); tft.print(F("Speed"));
  tft.setCursor(90, 203); tft.print(F("Depth"));

  b01Toggle.attach(B01_PIN, INPUT_PULLUP); // USE EXTERNAL PULL-UP
  b01Toggle.interval(5);
  b01Toggle.setPressedState(0);
  
  b02Toggle.attach(B02_PIN, INPUT_PULLUP); // USE EXTERNAL PULL-UP
  b02Toggle.interval(5);
  b02Toggle.setPressedState(0);

  b03Toggle.attach(B03_PIN, INPUT_PULLUP); // USE EXTERNAL PULL-UP
  b03Toggle.interval(5);
  b03Toggle.setPressedState(0);
  
  b04Toggle.attach(B04_PIN, INPUT_PULLUP); // USE EXTERNAL PULL-UP
  b04Toggle.interval(5);
  b04Toggle.setPressedState(0);

  b05Toggle.attach(B05_PIN, INPUT_PULLUP); // USE EXTERNAL PULL-UP
  b05Toggle.interval(5);
  b05Toggle.setPressedState(0);
  
  b06Toggle.attach(B06_PIN, INPUT_PULLUP); // USE EXTERNAL PULL-UP
  b06Toggle.interval(5);
  b06Toggle.setPressedState(0);

  b07momentary.attach(B07_PIN, INPUT_PULLUP); // USE EXTERNAL PULL-UP
  b07momentary.interval(250);
  b07momentary.setPressedState(0);
  
  // Set Starting notes here:
  r01.setPosition(40 / r01STEPS); // Starts at E2
  r02.setPosition(42 / r02STEPS); // Starts at F#2
  r03.setPosition(43 / r03STEPS); // Starts at G2
  r04.setPosition(45 / r04STEPS); // Starts at A2

  menuCheck();  // Sets the menu mode, detects if the extender unit is plugged in.
        if (b07 == 0) {
      Serial.println("0");
      FourButtonMenu();
      }
      else if (b07 == 1) {
      Serial.println("1");
      TwoButtonMenu();
      }

} // END Setup



void loop() {
  b01Toggle.update();
  b02Toggle.update();
  b03Toggle.update();
  b04Toggle.update();
  b05Toggle.update();
  b06Toggle.update();
  b07momentary.update();
  
  r01.tick();  // Key
  r02.tick();  // Key
  r03.tick();  // Key
  r04.tick();  // Key

 menuCheck();

  // get the current physical position and calc the logical position
  int r01newPos = r01.getPosition() * r01STEPS;
  int r02newPos = r02.getPosition() * r02STEPS;
  int r03newPos = r03.getPosition() * r03STEPS;
  int r04newPos = r04.getPosition() * r04STEPS;

  r01Data = r01newPos;
  r02Data = r02newPos;
  r03Data = r03newPos;
  r04Data = r04newPos;

  if (b07 == 1) {
    if (r01newPos < r01MIN) { r01.setPosition(r01MIN / r01STEPS); r01newPos = r01MIN; } else if (r01newPos > r01MAX) { r01.setPosition(r01MAX / r01STEPS); r01newPos = r01MAX; } 
    if (r01lastPos != r01newPos) {r01lastPos = r01newPos; transpose1 = (freq[r01newPos]); tft.setCursor(8, 205);  tft.setTextSize(4); tft.setTextColor(WHITE, DARKGREY); tft.println(Key1[r01newPos]);}

    if (r02newPos < r02MIN) { r02.setPosition(r02MIN / r02STEPS); r02newPos = r02MIN; } else if (r02newPos > r02MAX) { r02.setPosition(r02MAX / r02STEPS);  r02newPos = r02MAX;  } 
    if (r02lastPos != r02newPos) {r02lastPos = r02newPos; transpose2 = (freq[r02newPos]); tft.setCursor(244, 205); tft.setTextSize(4); tft.setTextColor(WHITE, DARKGREY); tft.println(Key1[r02newPos]);}
  }

  if (b07 == 0) {
    if (r01newPos < r01MIN) {r01.setPosition(r01MIN / r01STEPS); r01newPos = r01MIN; } else if (r01newPos > r01MAX) { r01.setPosition(r01MAX / r01STEPS); r01newPos = r01MAX;}
    if (r01lastPos != r01newPos) {r01lastPos = r01newPos; transpose1 = (freq[r01newPos]); tft.setCursor(8, 183);  tft.setTextSize(2); tft.setTextColor(WHITE, DARKGREY);  tft.println(Key1[r01newPos]);}
    if (r02newPos < r02MIN) {r02.setPosition(r02MIN / r02STEPS); r02newPos = r02MIN; } else if (r02newPos > r02MAX) { r02.setPosition(r02MAX / r02STEPS); r02newPos = r02MAX; }
    if (r02lastPos != r02newPos) { r02lastPos = r02newPos; transpose2 = (freq[r02newPos]); tft.setCursor(244, 183); tft.setTextSize(2); tft.setTextColor(WHITE, DARKGREY);  tft.println(Key1[r02newPos]); }
  }

  if (r03newPos < r03MIN) {r03.setPosition(r03MIN / r03STEPS); r03newPos = r03MIN; } else if (r03newPos > r03MAX) {r03.setPosition(r03MAX / r03STEPS); r03newPos = r03MAX; } // if
  if (r03lastPos != r03newPos) {r03lastPos = r03newPos; transpose3 = (freq[r03newPos]); tft.setCursor(8, 215);   tft.setTextSize(2); tft.setTextColor(WHITE, DARKGREY); tft.println(Key1[r03newPos]);
   } // END r01

  if (r04newPos < r04MIN) {r04.setPosition(r04MIN / r04STEPS); r04newPos = r04MIN;} else if (r04newPos > r04MAX) {r04.setPosition(r04MAX / r04STEPS); r04newPos = r04MAX;  } // if
  if (r04lastPos != r04newPos) {r04lastPos = r04newPos; transpose4 = (freq[r04newPos]); tft.setCursor(244, 215); tft.setTextSize(2); tft.setTextColor(WHITE, DARKGREY); tft.println(Key1[r04newPos]);
  } // END r02


    // Buttons:
  if ( b01Toggle.pressed() ) {
      b01 = !b01; 
      if (b01 == 1){
        noteOffA();
        }
      if (b01 == 0) {
        transpose1 = (freq[r01newPos]);
        noteOnA();  
      } }

  if ( b02Toggle.pressed() ) {
    b02 = !b02;
    if (b02 == 1){
        noteOffB();
    
        }
      if (b02 == 0)  {
      transpose1 = (freq[r01newPos]);
      noteOnB();
    
        }
  } 

     if (b03Toggle.fell()) {
          presetIndex--;
      if (presetIndex < 0) {
        presetIndex = arraySize - 1;
      }
      preset = (presetNum[presetIndex]);
      presetMenu();
      noteOffA();
      noteOffB();
    }
    
    if (b04Toggle.fell()) {
      presetIndex++;
      if (presetIndex >= arraySize) {
        presetIndex = 0;}
      preset = (presetNum[presetIndex]);
      presetMenu();
      noteOffA();
      noteOffB();
    }
  
    
  if ( b05Toggle.pressed() ) {
      b05 = !b05; 
      if (b05 == 1){
        noteOffC();
        }
      if (b05 == 0) {
        transpose3 = (freq[r03newPos]);
        noteOnC();  
        }
      }

  if ( b06Toggle.pressed() ) {
    b06 = !b06;
    if (b06 == 1){
        noteOffD();
        }
      if (b06 == 0)  {
       transpose4 = (freq[r04newPos]);
       noteOnD();
      }
    } 

 if (Touch.touched()) {
   updateSliders();

 } 
} 

void updateSliders () {
 
  ProcessTouch();
  tft.setTextSize(1);
  tft.setTextColor(WHITE, DARKGREY);

  bool changed1 = sBand1.slide(BtnX, BtnY);
  bool changed2 = sBand2.slide(BtnX, BtnY);
  bool changed3 = sBand3.slide(BtnX, BtnY);
  bool changed4 = sBand4.slide(BtnX, BtnY);
  bool changed5 = sBand5.slide(BtnX, BtnY);
  bool changed6 = sBand6.slide(BtnX, BtnY);
  bool changed7 = sBand7.slide(BtnX, BtnY);
  bool changed8 = sBand8.slide(BtnX, BtnY);
  bool changed9 = sBand9.slide(BtnX, BtnY);
  bool changed10 = sBand10.slide(BtnX, BtnY);

  if (changed1) {
   //            x   y   w   h   color
   tft.fillRect(14, 28, 18, 10, BACKCOLOR);  // detune
   tft.setCursor(14 , 30 ); tft.print(sBand1.value, 0); 
   detuneFactor = 1 - (0.05 * (sBand1.value * DIV127));
   }
   
  if (changed2) {  // octave
   tft.fillRect(55, 28, 18, 10, BACKCOLOR);
   tft.setCursor(55, 30 ); 
   octCase = sBand2.value;
   octSwitch ();
   tft.print(octType[octCase]);
   }

  if (changed3) { // FREQ
   tft.fillRect(95, 28, 18, 10, BACKCOLOR);
   freqControl = sBand3.value;
   tft.setCursor(95 , 30 ); tft.print(sBand3.value, 0); 
   filter1.frequency(10000 * (freqControl * DIV127)); 
   filter2.frequency(10000 * (freqControl * DIV127)); 
   }
 
  if (changed4) { //REZ
   tft.fillRect(135, 28, 18, 10, BACKCOLOR);
   rezControl = sBand4.value;
   tft.setCursor(135 , 30 ); tft.print(sBand4.value, 0);
   filter1.resonance((4.3 * (rezControl * DIV127)) + 0.7);
   filter2.resonance((4.3 * (rezControl * DIV127)) + 0.7);
   }
   
  if (changed5) { // Roomsize
   tft.fillRect(BAND5, 28, 18, 10, BACKCOLOR);  
   sizeSet = (sBand5.value / 127);
   freeverb1.roomsize(sizeSet);
   tft.setCursor(BAND5 , 30 ); tft.print(sBand5.value, 0);
   }

  if (changed6) { // Dampening
   tft.fillRect(BAND6, 28, 18, 10, BACKCOLOR); 
   dampSet = (sBand6.value / 127);
   freeverb1.damping(dampSet);
   tft.setCursor(BAND6 , 30 ); tft.print(sBand6.value, 0);
   }

  if (changed7) { // Dry
   tft.fillRect(BAND7, 28, 18, 10, BACKCOLOR); 
   tft.setCursor(BAND7 , 30 ); tft.print(sBand7.value, 0);
   dryVolume = (0.7 * (sBand7.value * DIV127));
   mixer2.gain(0, dryVolume);
   }
    
  if (changed8) { // Wet
   tft.fillRect(294, 28, 18, 10, BACKCOLOR);           
   tft.setCursor(294 , 30 ); tft.print(sBand8.value, 0);
   wetVolume = (0.7 * (sBand8.value * DIV127));
   mixer2.gain(1, wetVolume);
   }

  if (changed9) {   // Speed - LFO
   tft.fillRect(BAND9, 190, 18, 10, BACKCOLOR);
   tft.setCursor(BAND9 , 190 ); tft.print(sBand9.value, 0); 
   LFOspeed = sBand9.value; lfo.frequency(3 * (LFOspeed * DIV127)); 
   }


  if (changed10) { // Depth - LFO
   tft.fillRect(BAND9, 210, 18, 10, BACKCOLOR);
   tft.setCursor(BAND9 , 212 ); tft.print(sBand10.value, 0);
   LFOdepth = sBand10.value; lfo.amplitude(0.99 * (LFOdepth * DIV127)); 
   }


}
void ProcessTouch() {
  TouchPoint = Touch.getPoint();
  BtnX = TouchPoint.x;
  BtnY = TouchPoint.y;
  BtnX = map(BtnX, 362, 3865, 0, 320);
  BtnY = map(BtnY, 251, 3884, 0, 240); 
  }
