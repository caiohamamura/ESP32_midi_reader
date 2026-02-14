/*
 * ESP32 DevKit V1 18-Channel MIDI Player with DAC Output
 * Static allocation version with per-track event limiting
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

// Configuration - Adjust these based on available memory
#define MAX_TRACKS 18           // Maximum number of tracks to support
#define MAX_POLYPHONY 18        // Maximum simultaneous notes
#define MAX_EVENTS_PER_TRACK 800 // Events per track (static allocation)
#define TOTAL_EVENT_POOL 6000   // Total events across ALL tracks

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

// MIDI Event structure (16 bytes)
struct MIDIEvent {
  uint32_t deltaTime;
  uint32_t absoluteTime;
  uint8_t type;
  uint8_t channel;
  uint8_t note;
  uint8_t velocity;
};

// Track structure - points into shared event pool
struct Track {
  uint16_t eventStart;      // Starting index in global event pool
  uint16_t eventCount;      // Number of events
  uint16_t currentEvent;    // Current playback position
  bool active;              // Is this track being used?
};

// Active note structure for polyphony
struct ActiveNote {
  uint8_t note;
  uint8_t channel;
  uint8_t instrument;  // MIDI program number
  float frequency;
  uint8_t velocity;
  bool active;
};

// Global variables - static allocation
Track tracks[MAX_TRACKS];
MIDIEvent eventPool[TOTAL_EVENT_POOL];  // Shared pool of events
uint16_t nextEventIndex = 0;            // Next free slot in event pool

uint8_t channelInstruments[16] = {0};   // Track instrument per MIDI channel (0-15)

uint8_t trackCount = 0;
uint8_t polyphonyCount = MAX_POLYPHONY;
uint16_t ticksPerQuarter = 480;
uint32_t microsecondsPerQuarter = 500000; // Default 120 BPM
ActiveNote DRAM_ATTR activeNotes[MAX_POLYPHONY];
uint32_t currentTick = 0;

// Audio generation
volatile float DRAM_ATTR phase[MAX_POLYPHONY];

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
  
  // Limit to MAX_TRACKS
  trackCount = min(numTracks, (uint16_t)MAX_TRACKS);
  if (numTracks > MAX_TRACKS) {
    Serial.printf("Warning: MIDI has %d tracks, limiting to %d\n", numTracks, MAX_TRACKS);
  }
  
  // Reset event pool
  nextEventIndex = 0;
  
  // Initialize all tracks
  for (int t = 0; t < MAX_TRACKS; t++) {
    tracks[t].eventStart = 0;
    tracks[t].eventCount = 0;
    tracks[t].currentEvent = 0;
    tracks[t].active = false;
  }
  
  Serial.printf("Event pool capacity: %d events (%d bytes)\n", 
                TOTAL_EVENT_POOL, TOTAL_EVENT_POOL * sizeof(MIDIEvent));
  
  // Parse each track
  for (int t = 0; t < numTracks; t++) {
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
    
    Serial.printf("Track %d length: %d bytes", t, trackLength);
    
    uint32_t absoluteTime = 0;
    uint8_t runningStatus = 0;
    
    // Only parse events if this track is within our limit
    bool shouldParse = (t < trackCount);
    
    // Mark starting point in event pool
    if (shouldParse) {
      tracks[t].eventStart = nextEventIndex;
      tracks[t].active = true;
    }
    
    uint16_t trackEventCount = 0;
    uint16_t skippedEvents = 0;
    
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
        
        // Store if we're parsing this track and have space
        bool canStore = shouldParse && 
                       nextEventIndex < TOTAL_EVENT_POOL &&
                       trackEventCount < MAX_EVENTS_PER_TRACK;
        
        if (canStore) {
          eventPool[nextEventIndex].deltaTime = deltaTime;
          eventPool[nextEventIndex].absoluteTime = absoluteTime;
          eventPool[nextEventIndex].type = type;
          eventPool[nextEventIndex].channel = channel;
          eventPool[nextEventIndex].note = note;
          eventPool[nextEventIndex].velocity = velocity;
          nextEventIndex++;
          trackEventCount++;
        } else if (shouldParse) {
          skippedEvents++;
        }
        
      } else if (type == MIDI_PROGRAM_CHANGE) {
        if (file.available() < 1) break;
        uint8_t program = file.read();
        
        // Track instrument change for this channel
        channelInstruments[channel] = program;
        
        if (shouldParse) {
          Serial.printf("\n  Ch%d: Program %d", channel, program);
        }
      } else if (type == 0xB0) { // Control Change
        if (file.available() < 2) break;
        file.read();
        file.read();
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
          Serial.printf("\nTempo: %d us/quarter\n", microsecondsPerQuarter);
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
    
    if (shouldParse) {
      tracks[t].eventCount = trackEventCount;
      Serial.printf(" → %d events", trackEventCount);
      if (skippedEvents > 0) {
        Serial.printf(" (skipped %d)", skippedEvents);
      }
      Serial.println();
    } else {
      Serial.println(" (not parsed)");
    }
  }
  
  Serial.printf("\nTotal events loaded: %d / %d (%.1f%% full)\n", 
                nextEventIndex, TOTAL_EVENT_POOL, 
                (nextEventIndex * 100.0) / TOTAL_EVENT_POOL);
  Serial.printf("Memory used: %d bytes for events\n", nextEventIndex * sizeof(MIDIEvent));
  
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
  for (int t = 0; t < trackCount; t++) {
    if (!tracks[t].active) continue;
    
    Track &track = tracks[t];
    
    while (track.currentEvent < track.eventCount) {
      uint16_t eventIndex = track.eventStart + track.currentEvent;
      MIDIEvent &evt = eventPool[eventIndex];
      
      if (evt.absoluteTime > currentTick) {
        break;
      }
      
      if (evt.type == MIDI_NOTE_ON) {
        // Find free slot or same note
        int slot = -1;
        for (int i = 0; i < polyphonyCount; i++) {
          if (!activeNotes[i].active || 
              (activeNotes[i].note == evt.note && activeNotes[i].channel == evt.channel)) {
            slot = i;
            break;
          }
        }
        
        // If no free slot, steal oldest (first in array)
        if (slot < 0) {
          slot = 0;
        }
        
        if (slot >= 0) {
          activeNotes[slot].note = evt.note;
          activeNotes[slot].channel = evt.channel;
          activeNotes[slot].instrument = channelInstruments[evt.channel];
          activeNotes[slot].frequency = getNoteFrequency(evt.note);
          activeNotes[slot].velocity = evt.velocity;
          activeNotes[slot].active = true;
          phase[slot] = 0;
        }
        
      } else if (evt.type == MIDI_NOTE_OFF) {
        // Stop the note
        for (int i = 0; i < polyphonyCount; i++) {
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

  Serial.println("ESP32 DevKit V1 18-Track MIDI Player Starting...");
  Serial.printf("Configuration:\n");
  Serial.printf("  MAX_TRACKS: %d\n", MAX_TRACKS);
  Serial.printf("  MAX_POLYPHONY: %d voices\n", MAX_POLYPHONY);
  Serial.printf("  MAX_EVENTS_PER_TRACK: %d\n", MAX_EVENTS_PER_TRACK);
  Serial.printf("  TOTAL_EVENT_POOL: %d events\n", TOTAL_EVENT_POOL);
  Serial.printf("  Memory for events: %d bytes\n", TOTAL_EVENT_POOL * sizeof(MIDIEvent));
  Serial.printf("  Initial free heap: %d bytes\n\n", ESP.getFreeHeap());
  
  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed");
    return;
  }
  Serial.println("LittleFS Mounted");
  
  // Initialize DAC
  dac_output_enable(DAC_CHANNEL);
  dac_output_voltage(DAC_CHANNEL, 128);
  Serial.printf("DAC initialized on GPIO%d\n", DAC_GPIO);

  // Setup timer for audio sample generation (16kHz)
  audioTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(audioTimer, &onAudioTimer, true);
  timerAlarmWrite(audioTimer, 62, true);
  timerAlarmEnable(audioTimer);
  
  // Initialize active notes
  for (int i = 0; i < MAX_POLYPHONY; i++) {
    activeNotes[i].active = false;
    activeNotes[i].frequency = 0;
    activeNotes[i].velocity = 0;
    activeNotes[i].note = 0;
    activeNotes[i].channel = 0;
    activeNotes[i].instrument = 0;
    phase[i] = 0;
  }
  
  // Initialize channel instruments (default to Acoustic Grand Piano)
  for (int i = 0; i < 16; i++) {
    channelInstruments[i] = 0;
  }
  channelInstruments[9] = 128; // Channel 10 (index 9) is percussion
  
  Serial.println("Active notes initialized");
  
  // Parse MIDI file
  Serial.println("\nParsing MIDI file...");
  if (!parseMIDI("/data/overworld.mid")) {
    Serial.println("Failed to parse MIDI file");
    return;
  }
  
  Serial.println("\nMIDI parsed successfully!");
  Serial.printf("Active tracks: %d\n", trackCount);
  Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());
  
  // Wait before starting playback
  delay(500);
  
  Serial.println("\nStarting playback...\n");

  uint32_t start = micros();
  lastSample = start;
  lastTickTime = start;
}

void generateSample() {
  int16_t sample = 0;

  for (int i = 0; i < polyphonyCount; i++) {
    if (activeNotes[i].active) {

      if (activeNotes[i].channel == 9) {
        // Percussion channel - use noise
        int noiseVal = (random(256) - 128);
        sample += (noiseVal * activeNotes[i].velocity) >> 9;
      } else {
        // Melodic channel - select waveform based on instrument
        float freq = activeNotes[i].frequency;
        float increment = freq / SAMPLE_RATE;
        uint8_t instrument = activeNotes[i].instrument;

        phase[i] += increment;
        if (phase[i] >= 1.0f) phase[i] -= 1.0f;

        int16_t wave = 0;
        
        // Different waveforms for different instrument families
        // Piano (0-7): Bright square wave
        if (instrument <= 7) {
          wave = (phase[i] < 0.5f) ? 127 : -128;
        }
        // Chromatic Percussion (8-15): Triangle wave
        else if (instrument <= 15) {
          wave = (phase[i] < 0.5f) ? (phase[i] * 512 - 128) : (384 - phase[i] * 512);
        }
        // Organ (16-23): Square wave with harmonics
        else if (instrument <= 23) {
          wave = (phase[i] < 0.5f) ? 127 : -128;
          // Add some 2nd harmonic
          float phase2 = fmod(phase[i] * 2.0f, 1.0f);
          wave += ((phase2 < 0.5f) ? 32 : -32);
        }
        // Guitar (24-31): Sawtooth
        else if (instrument <= 31) {
          // Calculate time since note started (in samples)
          float timeSinceStart = phase[i] * (SAMPLE_RATE / freq);
          
          // Exponential decay envelope (fast attack, slow decay)
          float envelope = exp(-timeSinceStart * 0.003f); 
          
          wave = ((phase[i] * 255) - 128) * envelope;
        }
        // Bass (32-39): Deep square wave
        else if (instrument <= 39) {
          wave = (phase[i] < 0.5f) ? 100 : -100;
        }
        // Strings (40-47): Soft triangle
        else if (instrument <= 47) {
          wave = (phase[i] < 0.5f) ? (phase[i] * 384 - 96) : (288 - phase[i] * 384);
        }
        // Ensemble (48-55): Triangle with vibrato
        else if (instrument <= 55) {
          wave = (phase[i] < 0.5f) ? (phase[i] * 448 - 112) : (336 - phase[i] * 448);
        }
        // Brass (56-63): Rich square
        else if (instrument <= 63) {
          wave = (phase[i] < 0.4f) ? 127 : -128;
        }
        // Reed/Woodwinds (64-79): Clarinet uses hollow square wave
        else if (instrument <= 79) {
          // Hollow square wave (like clarinet - odd harmonics only)
          wave = (phase[i] < 0.3f || (phase[i] > 0.5f && phase[i] < 0.8f)) ? 90 : -90;
        }
        // Pipe (80-87): Sine-like (approximated with multi-step)
        else if (instrument <= 87) {
          if (phase[i] < 0.25f) wave = phase[i] * 512;
          else if (phase[i] < 0.75f) wave = 128 - (phase[i] - 0.25f) * 512;
          else wave = -128 + (phase[i] - 0.75f) * 512;
        }
        // Synth Lead (80-95): Sawtooth
        else if (instrument <= 95) {
          wave = (phase[i] * 255) - 128;
        }
        // Synth Pad (96-103): Soft triangle
        else if (instrument <= 103) {
          wave = (phase[i] < 0.5f) ? (phase[i] * 320 - 80) : (240 - phase[i] * 320);
        }
        // Synth Effects (104-111): Modulated square
        else if (instrument <= 111) {
          float mod = sin(phase[i] * 6.28318f) * 0.3f;
          wave = ((phase[i] + mod) < 0.5f) ? 110 : -110;
        }
        // Ethnic (112-119): Variable square
        else if (instrument <= 119) {
          wave = (phase[i] < 0.4f) ? 100 : -100;
        }
        // Percussive (120-127): Damped square
        else {
          wave = (phase[i] < 0.5f) ? 80 : -80;
        }
        
        sample += (wave * activeNotes[i].velocity) >> 7;
      }
    }
  }

  // Adjust mixing based on polyphony count
  int shift = 2;
  if (polyphonyCount > 8) shift = 3;
  if (polyphonyCount > 12) shift = 4;
  
  sample >>= shift;
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
  for (int i = 0; i < polyphonyCount; i++) {
    activeNotes[i].active = false;
    phase[i] = 0;
  }
  
  Serial.println("♪ Looping...");
}

void loop() {
  uint32_t now = micros();

  // Generate all missed samples
  while ((now - lastSample) >= 62) {
    lastSample += 62;
    generateSample();
  }

  // Process MIDI ticks
  uint32_t usPerTick = microsecondsPerQuarter / ticksPerQuarter;

  while ((now - lastTickTime) >= usPerTick) {
    lastTickTime += usPerTick;
    processTick();
    currentTick++;
  }

  // Loop detection
  bool allFinished = true;

  for (int t = 0; t < trackCount; t++) {
    if (tracks[t].active && tracks[t].currentEvent < tracks[t].eventCount) {
      allFinished = false;
      break;
    }
  }

  if (allFinished) {
    restartPlayback();
  }
}
