/*
 * ESP32 DevKit V1 4-Channel MIDI Player with DAC Output
 * Plays Super Mario Bros Overworld Theme
 * Outputs to GPIO25 (DAC1) or GPIO26 (DAC2)
 */

#include <Arduino.h>
#include "FS.h"
#include "LittleFS.h"
#include <WiFi.h>
#include <driver/dac.h>

// GPIO Pin for audio output - ESP32 DevKit V1 DAC pins
#define DAC_CHANNEL DAC_CHANNEL_1  // GPIO25 (DAC1) or use DAC_CHANNEL_2 for GPIO26
#define DAC_GPIO 25  // GPIO25 for DAC1, or 26 for DAC2

// MIDI Parser Constants
#define MAX_EVENTS 1000  // Reduced for DevKit V1 limited RAM

// MIDI Event Types
#define MIDI_NOTE_OFF 0x80
#define MIDI_NOTE_ON 0x90
#define MIDI_PROGRAM_CHANGE 0xC0
#define MIDI_META_EVENT 0xFF
#define MIDI_SET_TEMPO 0x51
#define MIDI_END_OF_TRACK 0x2F

// Audio generation
#define SAMPLE_RATE 16000

// Note frequency table in PROGMEM (only commonly used range)
const float PROGMEM noteFreqTable[] = {
  // MIDI notes 21-108 (A0 to C8)
  27.50, 29.14, 30.87, 32.70, 34.65, 36.71, 38.89, 41.20, 43.65, 46.25, 49.00, 51.91,
  55.00, 58.27, 61.74, 65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.83,
  110.00, 116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 196.00, 207.65,
  220.00, 233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392.00, 415.30,
  440.00, 466.16, 493.88, 523.25, 554.37, 587.33, 622.25, 659.25, 698.46, 739.99, 783.99, 830.61,
  880.00, 932.33, 987.77, 1046.50, 1108.73, 1174.66, 1244.51, 1318.51, 1396.91, 1479.98, 1567.98, 1661.22,
  1760.00, 1864.66, 1975.53, 2093.00, 2217.46, 2349.32, 2489.02, 2637.02, 2793.83, 2959.96, 3135.96, 3322.44,
  3520.00, 3729.31, 3951.07, 4186.01
};

// Get note frequency from table
float IRAM_ATTR getNoteFrequency(uint8_t note) {
  if (note < 21) return 27.50; // Return A0 for very low notes
  if (note > 108) return 4186.01; // Return C8 for very high notes
  return pgm_read_float(&noteFreqTable[note - 21]);
}

// MIDI Event structure
struct MIDIEvent {
  uint32_t deltaTime;
  uint32_t absoluteTime;
  uint8_t type;
  uint8_t channel;
  uint8_t note;
  uint8_t velocity;
};

// Track structure
struct Track {
  MIDIEvent events[MAX_EVENTS];
  uint16_t eventCount;
  uint16_t currentEvent;
};

// Active note structure for polyphony
struct ActiveNote {
  uint8_t note;
  uint8_t channel;
  float frequency;
  uint8_t velocity;
  bool active;
};

// Global variables
Track *tracks = nullptr;
uint8_t trackCount = 0;
uint16_t ticksPerQuarter = 480;
uint32_t microsecondsPerQuarter = 500000; // Default 120 BPM
ActiveNote DRAM_ATTR activeNotes[4]; // 4 channels for polyphony
uint32_t currentTick = 0;

// Audio generation - no hardware timer needed
volatile float DRAM_ATTR phase[4] = {0, 0, 0, 0};

// Read variable length quantity from MIDI file
uint32_t readVarLen(File &file) {
  uint32_t value = 0;
  uint8_t byte;
  
  do {
    byte = file.read();
    value = (value << 7) | (byte & 0x7F);
  } while (byte & 0x80);
  
  return value;
}

// Read 16-bit big-endian
uint16_t read16BE(File &file) {
  uint16_t value = file.read() << 8;
  value |= file.read();
  return value;
}

// Read 32-bit big-endian
uint32_t read32BE(File &file) {
  uint32_t value = file.read() << 24;
  value |= file.read() << 16;
  value |= file.read() << 8;
  value |= file.read();
  return value;
}

// Parse MIDI file
bool parseMIDI(const char* filename) {
  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println("Failed to open MIDI file");
    return false;
  }
  
  // Read header chunk
  char headerID[5] = {0};
  file.readBytes(headerID, 4);
  if (strcmp(headerID, "MThd") != 0) {
    Serial.println("Invalid MIDI header");
    file.close();
    return false;
  }
  
  uint32_t headerLength = read32BE(file);
  uint16_t format = read16BE(file);
  uint16_t numTracks = read16BE(file);
  ticksPerQuarter = read16BE(file);
  
  Serial.printf("MIDI Format: %d, Tracks: %d, TPQ: %d\n", format, numTracks, ticksPerQuarter);
  
  // Allocate for actual number of tracks
  trackCount = numTracks;
  
  // Allocate tracks in heap memory
  if (tracks != nullptr) {
    free(tracks);
  }
  tracks = (Track*)malloc(sizeof(Track) * trackCount);
  if (tracks == nullptr) {
    Serial.println("Failed to allocate memory for tracks");
    file.close();
    return false;
  }
  
  Serial.printf("Allocated %d bytes for %d tracks\n", sizeof(Track) * trackCount, trackCount);

  // Remaining memory
  Serial.printf("Free heap after track allocation: %d bytes\n", ESP.getFreeHeap());
  
  // Initialize all tracks
  for (int t = 0; t < trackCount; t++) {
    tracks[t].eventCount = 0;
    tracks[t].currentEvent = 0;
  }
  
  // Parse each track
  for (int t = 0; t < trackCount; t++) {
    // Check if we have enough data
    if (file.available() < 8) {
      Serial.printf("Not enough data for track %d\n", t);
      break;
    }
    
    char trackID[5] = {0};
    file.readBytes(trackID, 4);
    
    if (strcmp(trackID, "MTrk") != 0) {
      Serial.printf("Invalid track %d header: %s\n", t, trackID);
      // Try to recover by searching for next MTrk
      bool found = false;
      while (file.available() >= 4) {
        char c = file.read();
        if (c == 'M') {
          char temp[4];
          temp[0] = 'M';
          file.readBytes(&temp[1], 3);
          if (strcmp(temp, "MTrk") == 0) {
            found = true;
            Serial.printf("Found MTrk after recovery\n");
            break;
          } else {
            file.seek(file.position() - 3);
          }
        }
      }
      if (!found) {
        continue;
      }
    }
    
    uint32_t trackLength = read32BE(file);
    uint32_t trackEnd = file.position() + trackLength;
    
    Serial.printf("Track %d length: %d bytes\n", t, trackLength);
    
    uint32_t absoluteTime = 0;
    uint8_t runningStatus = 0;
    
    while (file.position() < trackEnd && file.available() > 0) {
      uint32_t deltaTime = readVarLen(file);
      absoluteTime += deltaTime;
      
      if (file.position() >= trackEnd) break;
      
      uint8_t status = file.read();
      
      // Handle running status
      if (status < 0x80) {
        file.seek(file.position() - 1);
        status = runningStatus;
      } else {
        runningStatus = status;
      }
      
      uint8_t type = status & 0xF0;
      uint8_t channel = status & 0x0F;
      
      if (type == MIDI_NOTE_ON || type == MIDI_NOTE_OFF) {
        if (file.available() < 2) break;
        
        uint8_t note = file.read();
        uint8_t velocity = file.read();
        
        // Treat NOTE_ON with velocity 0 as NOTE_OFF
        if (type == MIDI_NOTE_ON && velocity == 0) {
          type = MIDI_NOTE_OFF;
        }
        
        // Only store if we have space
        if (tracks[t].eventCount < MAX_EVENTS) {
          MIDIEvent &evt = tracks[t].events[tracks[t].eventCount];
          evt.deltaTime = deltaTime;
          evt.absoluteTime = absoluteTime;
          evt.type = type;
          evt.channel = channel;
          evt.note = note;
          evt.velocity = velocity;
          tracks[t].eventCount++;
        }
        
      } else if (type == MIDI_PROGRAM_CHANGE) {
        if (file.available() < 1) break;
        uint8_t program = file.read();
        // Ignore program changes for buzzer
      } else if (type == 0xB0) { // Control Change
        if (file.available() < 2) break;
        file.read(); // controller
        file.read(); // value
      } else if (type == 0xE0) { // Pitch Bend
        if (file.available() < 2) break;
        file.read();
        file.read();
      } else if (type == 0xD0) { // Channel Pressure
        if (file.available() < 1) break;
        file.read();
      } else if (type == 0xA0) { // Polyphonic Key Pressure
        if (file.available() < 2) break;
        file.read();
        file.read();
      } else if (status == MIDI_META_EVENT) {
        if (file.available() < 2) break;
        
        uint8_t metaType = file.read();
        uint32_t metaLength = readVarLen(file);
        
        if (metaType == MIDI_SET_TEMPO && metaLength == 3 && file.available() >= 3) {
          microsecondsPerQuarter = (file.read() << 16) | (file.read() << 8) | file.read();
          Serial.printf("Tempo: %d us/quarter\n", microsecondsPerQuarter);
        } else {
          // Skip other meta events
          for (uint32_t i = 0; i < metaLength && file.available() > 0; i++) {
            file.read();
          }
        }
      } else if (status == 0xF0 || status == 0xF7) { // SysEx
        uint32_t sysexLen = readVarLen(file);
        for (uint32_t i = 0; i < sysexLen && file.available() > 0; i++) {
          file.read();
        }
      }
    }
    
    // Make sure we're at the end of the track
    if (file.position() < trackEnd) {
      file.seek(trackEnd);
    }
    
    Serial.printf("Track %d: %d events\n", t, tracks[t].eventCount);
  }
  
  file.close();
  return true;
}

hw_timer_t *audioTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint8_t currentSample = 128;

// DAC output function called by timer interrupt
void IRAM_ATTR onAudioTimer() {
  dac_output_voltage(DAC_CHANNEL, currentSample);
}

// Process MIDI events for current tick
void processTick() {
  if (tracks == nullptr) return;
  
  for (int t = 0; t < trackCount; t++) {
    Track &track = tracks[t];
    
    while (track.currentEvent < track.eventCount) {
      MIDIEvent &evt = track.events[track.currentEvent];
      
      if (evt.absoluteTime > currentTick) {
        break;
      }
      
      if (evt.type == MIDI_NOTE_ON) {
        // Find free slot or same note
        int slot = -1;
        for (int i = 0; i < 4; i++) {
          if (!activeNotes[i].active || 
              (activeNotes[i].note == evt.note && activeNotes[i].channel == evt.channel)) {
            slot = i;
            break;
          }
        }
        
        if (slot >= 0) {
          activeNotes[slot].note = evt.note;
          activeNotes[slot].channel = evt.channel;
          activeNotes[slot].frequency = getNoteFrequency(evt.note);
          activeNotes[slot].velocity = evt.velocity;
          activeNotes[slot].active = true;
          phase[slot] = 0;
        }
        
      } else if (evt.type == MIDI_NOTE_OFF) {
        // Stop the note
        for (int i = 0; i < 4; i++) {
          if (activeNotes[i].active && 
              activeNotes[i].note == evt.note && 
              activeNotes[i].channel == evt.channel) {
            activeNotes[i].active = false;
            break;
          }
        }
      }
      
      track.currentEvent++;
    }
  }
}

uint32_t lastSample = 0;
uint32_t lastTickTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.println("ESP32 DevKit V1 MIDI Player with DAC Starting...");
  
  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS Mounted");
  
  // Initialize DAC
  dac_output_enable(DAC_CHANNEL);
  dac_output_voltage(DAC_CHANNEL, 128); // Set to middle value (0V relative to 0-3.3V range)
  Serial.printf("DAC initialized on GPIO%d\n", DAC_GPIO);

  // Setup timer for audio sample generation (16kHz = 62.5us per sample)
  audioTimer = timerBegin(0, 80, true);  // 80MHz / 80 = 1MHz (1us per tick)
  timerAttachInterrupt(audioTimer, &onAudioTimer, true);
  timerAlarmWrite(audioTimer, 62, true);  // 62us = ~16kHz sample rate
  timerAlarmEnable(audioTimer);
  
  // Initialize active notes
  for (int i = 0; i < 4; i++) {
    activeNotes[i].active = false;
    activeNotes[i].frequency = 0;
    activeNotes[i].velocity = 0;
    activeNotes[i].note = 0;
    activeNotes[i].channel = 0;
  }
  
  Serial.println("Active notes initialized");
  
  // Parse MIDI file
  if (!parseMIDI("/data/overworld.mid")) {
    Serial.println("Failed to parse MIDI file");
    return;
  }
  
  Serial.println("MIDI parsed successfully!");
  Serial.printf("Track count: %d\n", trackCount);
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  
  // Wait a bit before starting playback
  delay(500);
  
  Serial.println("Starting playback...");

  uint32_t start = micros();
  lastSample = start;
  lastTickTime = start;
}



void generateSample() {
  int16_t sample = 0;

  for (int i = 0; i < 4; i++) {
    if (activeNotes[i].active) {

      if (activeNotes[i].channel == 9) {
        int noiseVal = (random(256) - 128);
        sample += (noiseVal * activeNotes[i].velocity) >> 9;
      } else {
        float freq = activeNotes[i].frequency;
        float increment = freq / SAMPLE_RATE;

        phase[i] += increment;
        if (phase[i] >= 1.0f) phase[i] -= 1.0f;

        int16_t wave = (phase[i] < 0.5f) ? 127 : -128;
        sample += (wave * activeNotes[i].velocity) >> 7;
      }
    }
  }

  sample >>= 2;
  if (sample > 127) sample = 127;
  if (sample < -128) sample = -128;

  currentSample = sample + 128;
  
}

void restartPlayback() {

  currentTick = 0;

  // Reset track positions
  for (int t = 0; t < trackCount; t++) {
    tracks[t].currentEvent = 0;
  }

  // Stop all active notes
  for (int i = 0; i < 4; i++) {
    activeNotes[i].active = false;
    phase[i] = 0;
  }
}


void loop() {

  if (tracks == nullptr) return;


  uint32_t now = micros();

  // ---- Generate ALL missed samples ----
  while ((now - lastSample) >= 62) {
    lastSample += 62;
    generateSample();
  }

  // ---- Process MIDI ticks ----
  uint32_t usPerTick = microsecondsPerQuarter / ticksPerQuarter;

  while ((now - lastTickTime) >= usPerTick) {
    lastTickTime += usPerTick;
    processTick();
    currentTick++;
  }

  // ---- Loop detection ----
  bool allFinished = true;

  for (int t = 0; t < trackCount; t++) {
    if (tracks[t].currentEvent < tracks[t].eventCount) {
      allFinished = false;
      break;
    }
  }

  if (allFinished) {
    restartPlayback();
  }
}
