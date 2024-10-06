#include <arduinoFFT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>

arduinoFFT FFT;

// Constants
#define CHANNEL PA0
#define SAMPLES 128
#define SAMPLING_FREQUENCY 1000
#define NUM_MEASUREMENTS 30

// OLED display
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// Global variables
double vReal[SAMPLES];
double vImag[SAMPLES];
double dominantFrequencies[4];
double dominantAmplitudes[4];
double amplitudeBuffer[NUM_MEASUREMENTS];
int bufferIndex = 0;

// Functions
void setupOLED() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
}

void sampleData() {
  unsigned long microseconds = micros();
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(CHANNEL);
    vImag[i] = 0;
    while (micros() - microseconds < SAMPLING_FREQUENCY) {
      // Empty loop
    }
    microseconds += SAMPLING_FREQUENCY;
  }
}

void calculateFFT() {
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  for (int i = 0; i < 4; i++) {
    double maxAmplitude = 0;
    int maxIndex = 0;
    for (int j = 1; j < (SAMPLES >> 1); j++) {
      if (vReal[j] > maxAmplitude) {
        maxAmplitude = vReal[j];
        maxIndex = j;
      }
    }
    dominantFrequencies[i] = (maxIndex * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
    dominantAmplitudes[i] = maxAmplitude;
    vReal[maxIndex] = 0;
  }
}

void sortDominantFrequencies() {
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 4; j++) {
      if (dominantAmplitudes[i] < dominantAmplitudes[j]) {
        // Swap frequencies and amplitudes
        double tempFrequency = dominantFrequencies[i];
        dominantFrequencies[i] = dominantFrequencies[j];
        dominantFrequencies[j] = tempFrequency;

        double tempAmplitude = dominantAmplitudes[i];
        dominantAmplitudes[i] = dominantAmplitudes[j];
        dominantAmplitudes[j] = tempAmplitude;
      }
    }
  }
}

void calculateAverageAmplitude() {
  amplitudeBuffer[bufferIndex] = dominantAmplitudes[0];
  bufferIndex = (bufferIndex + 1) % NUM_MEASUREMENTS;
  double averageAmplitude = 0;
  for (int i = 0; i < NUM_MEASUREMENTS; i++) {
    averageAmplitude += amplitudeBuffer[i];
  }
  averageAmplitude /= NUM_MEASUREMENTS;
  // Use averageAmplitude as needed
}

void displayResults() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);

  // Display 1  dominant frequencies and amplitudes
  display.print("Major Frequencies:");
  for (int i = 0; i < 4; i++) {
    display.print("\n  - ");
    display.print(dominantFrequencies[i], 6);
    display.print(" Hz");
  }

  display.print("\nAmplitudes:");
  for (int i = 0; i < 4; i++) {
    display.print("\n  - ");
    display.print(dominantAmplitudes[i] / 10, 4);
  }
}

void sendResultsSerial() {
  // Send results to serial
  Serial.println("Dominant frequencies and amplitudes:");
  for (int i = 0; i < 4; i++) {
    Serial.print("Frequency " );
    Serial.print(i + 1);
    Serial.print(": ");
    //Serial.print(dominantFrequencies[i], 6);
    Serial.print(" Hz, Amplitude: ");
    Serial.println(dominantAmplitudes[i] / 10, 4);
  }
}

void loop() {
  sampleData();
  calculateFFT();
  sortDominantFrequencies();
  calculateAverageAmplitude();
  displayResults();
  sendResultsSerial();
}
