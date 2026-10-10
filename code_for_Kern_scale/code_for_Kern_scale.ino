
#include <Arduino.h>
#include <phyphoxBle.h>
#include <stdlib.h>
#include <string.h>

// ---------- Waage ----------
static constexpr uint32_t SCALE_BAUD = 9600;
static constexpr int SCALE_RX_PIN = 4;
static constexpr int SCALE_TX_PIN = 5;
static constexpr char DEVICE_NAME[] = "KERN Laborwaage";

float latestWeight = 0.0f;
bool weightStable = false;

// ---------- Zeilenpuffer ----------
bool readScale() {
  static char buffer[32];
  static size_t pos = 0;

  while (Serial1.available()) {
    char c = Serial1.read();

    if (c == '\r' || c == '\n') {
      if (pos == 0) continue;  // Leere Zeile ignorieren

      buffer[pos] = '\0';

      char* end;
      float value = strtof(buffer, &end);

      if (end == buffer) {
        pos = 0;
        continue;  // Keine gültige Zahl
      }

      latestWeight = value;
      weightStable = strchr(buffer, 'g') != nullptr || strchr(buffer, 'G') != nullptr;

      pos = 0;
      return true;
    }

    if (pos < sizeof(buffer) - 1) {
      buffer[pos++] = c;
    } else {
      pos = 0;  // Zu lange Zeile verwerfen
    }
  }

  return false;
}

// ---------- phyphox ----------
void setupPhyphox() {
  PhyphoxBleExperiment experiment;
  experiment.setTitle("KERN Laborwaage");
  experiment.setCategory("Labor");
  experiment.setDescription("Gewicht und Stabilität");
  experiment.setColor("32DB44");

  PhyphoxBleExperiment::View view;
  view.setLabel("Waage");

  PhyphoxBleExperiment::Graph weightGraph;
  weightGraph.setLabel("Gewichtsverlauf");
  weightGraph.setLabelX("Zeit");
  weightGraph.setLabelY("Gewicht (g)");
  weightGraph.setStyle(STYLE_LINES);
  weightGraph.setColor("32DB44");
  weightGraph.setXPrecision(1);
  weightGraph.setYPrecision(3);
  weightGraph.setChannel(0, 1);
  view.addElement(weightGraph);

  PhyphoxBleExperiment::Value weightValue;
  weightValue.setLabel("Aktuelles Gewicht (g)");
  weightValue.setPrecision(3);
  weightValue.setColor("32DB44");
  weightValue.setChannel(1);
  view.addElement(weightValue);

  PhyphoxBleExperiment::Value stabilityValue;
  stabilityValue.setLabel("Stabilität (1 = stabil, 0 = instabil)");
  stabilityValue.setPrecision(0);
  stabilityValue.setChannel(2);
  view.addElement(stabilityValue);

  experiment.addView(view);
  PhyphoxBLE::addExperiment(experiment);
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  Serial1.begin(
    SCALE_BAUD,
    SERIAL_8N1,
    SCALE_RX_PIN,
    SCALE_TX_PIN);

  setupPhyphox();
  PhyphoxBLE::start(DEVICE_NAME);
}

// ---------- Hauptschleife ----------
void loop() {
  if (readScale()) {
    float stability = weightStable ? 1.0f : 0.0f;

    PhyphoxBLE::write(latestWeight, stability);
  }

  PhyphoxBLE::poll();
}