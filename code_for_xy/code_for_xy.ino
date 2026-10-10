#include <Arduino.h>
#include <math.h>
#include <phyphoxBle.h>

// =============================================================================
// Konfiguration
// =============================================================================

static constexpr char PHYPHOX_DEVICE_NAME[] = "Tecpel DMM-8061 Dual";

static constexpr uint32_t DMM_BAUDRATE = 2400;
static constexpr uint8_t FRAME_SIZE = 14;
static constexpr uint16_t BUFFER_SIZE = FRAME_SIZE * 2;

static constexpr int DMM1_RX_PIN = 4;
static constexpr int DMM1_TX_PIN = 5;
static constexpr int DMM2_RX_PIN = 3;
static constexpr int DMM2_TX_PIN = 21;

// =============================================================================
// FS9721-Decoder
// =============================================================================

float latestValue1 = 0.0f;
float latestValue2 = 0.0f;

bool haveValue1 = false;
bool haveValue2 = false;

namespace FS9721 {

enum class Unit : uint8_t {
  None,
  Volt,
  Ampere,
  Ohm,
  Farad,
  Hertz,
  Percent,
  Diode,
  Continuity
};

enum class Prefix : int8_t {
  None = 0,
  Nano = -9,
  Micro = -6,
  Milli = -3,
  Kilo = 3,
  Mega = 6
};

struct Measurement {
  float value = NAN;

  Unit unit = Unit::None;
  Prefix prefix = Prefix::None;

  bool valid = false;
  bool overload = false;

  bool ac = false;
  bool dc = false;
  bool autoRange = false;
  bool rs232 = false;
  bool negative = false;

  bool diode = false;
  bool percent = false;
  bool beep = false;
  bool relative = false;
  bool hold = false;
  bool lowBattery = false;

  bool decimal1 = false;
  bool decimal2 = false;
  bool decimal3 = false;

  uint8_t userBits = 0;
};

inline uint8_t payload(uint8_t byte) {
  return byte & 0x0F;
}

inline bool isValidFrame(const uint8_t *frame) {
  for (uint8_t i = 0; i < FRAME_SIZE; ++i) {
    if ((frame[i] >> 4) != (i + 1)) {
      return false;
    }
  }

  return true;
}

inline int decodeDigit(uint8_t first, uint8_t second) {
  const uint8_t segments = static_cast<uint8_t>(
    ((payload(first) << 4) | payload(second)) & 0x7F);

  switch (segments) {
    case 0x7D: return 0;
    case 0x05: return 1;
    case 0x5B: return 2;
    case 0x1F: return 3;
    case 0x27: return 4;
    case 0x3E: return 5;
    case 0x7E: return 6;
    case 0x15: return 7;
    case 0x7F: return 8;
    case 0x3F: return 9;
    default: return -1;
  }
}

inline float prefixMultiplier(Prefix prefix) {
  switch (prefix) {
    case Prefix::Nano: return 1.0e-9f;
    case Prefix::Micro: return 1.0e-6f;
    case Prefix::Milli: return 1.0e-3f;
    case Prefix::Kilo: return 1.0e3f;
    case Prefix::Mega: return 1.0e6f;
    default: return 1.0f;
  }
}

inline bool decode(const uint8_t *frame, Measurement &result) {
  result = Measurement{};

  if (!isValidFrame(frame)) {
    return false;
  }

  const uint8_t b0 = payload(frame[0]);
  const uint8_t b9 = payload(frame[9]);
  const uint8_t b10 = payload(frame[10]);
  const uint8_t b11 = payload(frame[11]);
  const uint8_t b12 = payload(frame[12]);
  const uint8_t b13 = payload(frame[13]);

  // Betriebszustand
  result.ac = (b0 & 0x08) != 0;
  result.dc = (b0 & 0x04) != 0;
  result.autoRange = (b0 & 0x02) != 0;
  result.rs232 = (b0 & 0x01) != 0;

  result.negative = (payload(frame[1]) & 0x08) != 0;

  result.decimal1 = (payload(frame[3]) & 0x08) != 0;
  result.decimal2 = (payload(frame[5]) & 0x08) != 0;
  result.decimal3 = (payload(frame[7]) & 0x08) != 0;

  result.userBits = b13;

  // Statusbits
  result.percent = (b10 & 0x04) != 0;
  result.beep = (b10 & 0x01) != 0;
  result.relative = (b11 & 0x02) != 0;
  result.hold = (b11 & 0x01) != 0;
  result.lowBattery = (b12 & 0x01) != 0;
  result.diode = (b9 & 0x01) != 0;

  // SI-Präfix
  if (b9 & 0x08) {
    result.prefix = Prefix::Micro;
  } else if (b9 & 0x04) {
    result.prefix = Prefix::Nano;
  } else if (b10 & 0x08) {
    result.prefix = Prefix::Milli;
  } else if (b9 & 0x02) {
    result.prefix = Prefix::Kilo;
  } else if (b10 & 0x02) {
    result.prefix = Prefix::Mega;
  }

  // Einheit
  if (b12 & 0x04) {
    result.unit = Unit::Volt;
  } else if (b12 & 0x08) {
    result.unit = Unit::Ampere;
  } else if (b11 & 0x04) {
    result.unit = Unit::Ohm;
  } else if (b11 & 0x08) {
    result.unit = Unit::Farad;
  } else if (b12 & 0x02) {
    result.unit = Unit::Hertz;
  } else if (result.percent) {
    result.unit = Unit::Percent;
  } else if (result.diode) {
    result.unit = Unit::Diode;
  } else if (result.beep) {
    result.unit = Unit::Continuity;
  }

  // Overload prüfen; das Vorzeichenbit darf den Vergleich nicht stören.
  const uint8_t digit0 = static_cast<uint8_t>(
                           ((payload(frame[1]) & 0x07) << 4) | payload(frame[2]))
                         & 0x7F;

  const uint8_t digit1 = static_cast<uint8_t>(
                           (payload(frame[3]) << 4) | payload(frame[4]))
                         & 0x7F;

  const uint8_t digit2 = static_cast<uint8_t>(
                           (payload(frame[5]) << 4) | payload(frame[6]))
                         & 0x7F;

  const uint8_t digit3 = static_cast<uint8_t>(
                           (payload(frame[7]) << 4) | payload(frame[8]))
                         & 0x7F;

  if (digit0 == 0x00 && digit1 == 0x7D && digit2 == 0x68 && digit3 == 0x00) {
    result.overload = true;
    result.valid = true;
    result.value = result.negative ? -INFINITY : INFINITY;
    return true;
  }

  // Ziffern dekodieren
  const int d0 = decodeDigit(frame[1], frame[2]);
  const int d1 = decodeDigit(frame[3], frame[4]);
  const int d2 = decodeDigit(frame[5], frame[6]);
  const int d3 = decodeDigit(frame[7], frame[8]);

  if (d0 < 0 || d1 < 0 || d2 < 0 || d3 < 0) {
    return false;
  }

  float value =
    d0 * 1000.0f + d1 * 100.0f + d2 * 10.0f + d3;

  // Dezimalpunkt
  if (result.decimal1) {
    value *= 0.001f;
  } else if (result.decimal2) {
    value *= 0.01f;
  } else if (result.decimal3) {
    value *= 0.1f;
  }

  if (result.negative) {
    value = -value;
  }

  value *= prefixMultiplier(result.prefix);

  result.value = value;
  result.valid = true;
  return true;
}

inline const char *unitName(Unit unit) {
  switch (unit) {
    case Unit::Volt: return "V";
    case Unit::Ampere: return "A";
    case Unit::Ohm: return "Ohm";
    case Unit::Farad: return "F";
    case Unit::Hertz: return "Hz";
    case Unit::Percent: return "%";
    case Unit::Diode: return "V";
    case Unit::Continuity: return "Cont";
    default: return "";
  }
}

inline const char *prefixName(Prefix prefix) {
  switch (prefix) {
    case Prefix::Nano: return "n";
    case Prefix::Micro: return "u";
    case Prefix::Milli: return "m";
    case Prefix::Kilo: return "k";
    case Prefix::Mega: return "M";
    default: return "";
  }
}

}  // namespace FS9721

// =============================================================================
// DMM-Ports und Empfangspuffer
// =============================================================================

struct DmmPort {
  HardwareSerial *serial;
  int rxPin;
  int txPin;

  uint8_t buffer[BUFFER_SIZE];
  uint16_t bufferPos = 0;

  FS9721::Measurement latest;
};

DmmPort dmm1{ &Serial1, DMM1_RX_PIN, DMM1_TX_PIN };
DmmPort dmm2{ &Serial, DMM2_RX_PIN, DMM2_TX_PIN };

// =============================================================================
// phyphox-Experiment
// =============================================================================

PhyphoxBleExperiment phyphoxExperiment;

PhyphoxBleExperiment::View viewXY;
PhyphoxBleExperiment::Graph graphXY;
PhyphoxBleExperiment::Value valueX;
PhyphoxBleExperiment::Value valueY;

PhyphoxBleExperiment::View viewTime;
PhyphoxBleExperiment::Graph graphDmm1Time;
PhyphoxBleExperiment::Graph graphDmm2Time;
PhyphoxBleExperiment::Value valueDmm1;
PhyphoxBleExperiment::Value valueDmm2;

// =============================================================================
// DMM initialisieren
// =============================================================================

void initDmm(DmmPort &dmm) {
  dmm.serial->begin(
    DMM_BAUDRATE,
    SERIAL_8N1,
    dmm.rxPin,
    dmm.txPin);

  dmm.bufferPos = 0;
  dmm.latest = FS9721::Measurement{};
}

// =============================================================================
// Puffer verschieben
// =============================================================================

void discardBytes(DmmPort &dmm, uint16_t count) {
  if (count >= dmm.bufferPos) {
    dmm.bufferPos = 0;
    return;
  }

  for (uint16_t i = count; i < dmm.bufferPos; ++i) {
    dmm.buffer[i - count] = dmm.buffer[i];
  }

  dmm.bufferPos -= count;
}

// =============================================================================
// DMM pollen und einen vollständigen Frame dekodieren
// =============================================================================

bool pollDmm(DmmPort &dmm, FS9721::Measurement &result) {
  // Empfangsdaten in den Puffer übernehmen.
  while (dmm.serial->available() > 0 && dmm.bufferPos < BUFFER_SIZE) {
    dmm.buffer[dmm.bufferPos++] =
      static_cast<uint8_t>(dmm.serial->read());
  }

  if (dmm.bufferPos < FRAME_SIZE) {
    return false;
  }

  // Nach einem vollständig synchronisierten Frame suchen.
  for (uint16_t start = 0;
       start <= dmm.bufferPos - FRAME_SIZE;
       ++start) {

    if ((dmm.buffer[start] >> 4) != 0x01) {
      continue;
    }

    if (!FS9721::isValidFrame(&dmm.buffer[start])) {
      continue;
    }

    if (FS9721::decode(&dmm.buffer[start], result)) {
      discardBytes(dmm, start + FRAME_SIZE);
      dmm.latest = result;
      return true;
    }

    // Synchroner Frame, aber Inhalt nicht dekodierbar:
    // ab dem nächsten Byte weitersuchen.
    discardBytes(dmm, start + 1);
    return false;
  }

  // Noch kein kompletter Frame gefunden. Einen möglichen
  // unvollständigen Frame-Anfang im Puffer behalten.
  uint16_t keepFrom = dmm.bufferPos;

  for (uint16_t start = 0; start < dmm.bufferPos; ++start) {
    if ((dmm.buffer[start] >> 4) != 0x01) {
      continue;
    }

    bool prefixValid = true;

    for (uint16_t j = 0;
         j < FRAME_SIZE && start + j < dmm.bufferPos;
         ++j) {
      if ((dmm.buffer[start + j] >> 4) != (j + 1)) {
        prefixValid = false;
        break;
      }
    }

    if (prefixValid) {
      keepFrom = start;
      break;
    }
  }

  discardBytes(dmm, keepFrom);
  return false;
}

// =============================================================================
// phyphox-Experiment konfigurieren
// =============================================================================

void setupPhyphoxExperiment() {
  phyphoxExperiment.setTitle("Tecpel DMM-8061 Dual");
  phyphoxExperiment.setCategory("Multimeter");
  phyphoxExperiment.setDescription("Zwei Tecpel DMM-8061 Messwerte");
  phyphoxExperiment.setColor("32DB44");

  // Ansicht 1: DMM1 gegen DMM2
  viewXY.setLabel("XY-Plot");

  graphXY.setLabel("DMM1 vs DMM2");
  graphXY.setLabelX("DMM1");
  graphXY.setLabelY("DMM2");
  graphXY.setXPrecision(6);
  graphXY.setYPrecision(6);
  graphXY.setStyle(STYLE_LINES);
  graphXY.setColor("32DB44");
  graphXY.setLinewidth(1.0f);
  graphXY.setMinX(0, LAYOUT_AUTO);
  graphXY.setMaxX(0, LAYOUT_AUTO);
  graphXY.setMinY(0, LAYOUT_AUTO);
  graphXY.setMaxY(0, LAYOUT_AUTO);
  graphXY.setChannel(1, 2);
  viewXY.addElement(graphXY);

  valueX.setLabel("Aktueller Wert DMM1 (X)");
  valueX.setPrecision(6);
  valueX.setColor("32DB44");
  valueX.setChannel(1);
  viewXY.addElement(valueX);

  valueY.setLabel("Aktueller Wert DMM2 (Y)");
  valueY.setPrecision(6);
  valueY.setColor("E4572E");
  valueY.setChannel(2);
  viewXY.addElement(valueY);

  phyphoxExperiment.addView(viewXY);

  // Ansicht 2: Messwerte über die Zeit
  viewTime.setLabel("Zeit");

  graphDmm1Time.setLabel("DMM1 über Zeit");
  graphDmm1Time.setLabelX("Zeit");
  graphDmm1Time.setLabelY("DMM1");
  graphDmm1Time.setXPrecision(1);
  graphDmm1Time.setYPrecision(6);
  graphDmm1Time.setStyle(STYLE_LINES);
  graphDmm1Time.setColor("32DB44");
  graphDmm1Time.setLinewidth(2.0f);
  graphDmm1Time.setMinY(0, LAYOUT_AUTO);
  graphDmm1Time.setMaxY(0, LAYOUT_AUTO);
  graphDmm1Time.setChannel(0, 1);
  viewTime.addElement(graphDmm1Time);

  graphDmm2Time.setLabel("DMM2 über Zeit");
  graphDmm2Time.setLabelX("Zeit");
  graphDmm2Time.setLabelY("DMM2");
  graphDmm2Time.setXPrecision(1);
  graphDmm2Time.setYPrecision(6);
  graphDmm2Time.setStyle(STYLE_LINES);
  graphDmm2Time.setColor("E4572E");
  graphDmm2Time.setLinewidth(2.0f);
  graphDmm2Time.setMinY(0, LAYOUT_AUTO);
  graphDmm2Time.setMaxY(0, LAYOUT_AUTO);
  graphDmm2Time.setChannel(0, 2);
  viewTime.addElement(graphDmm2Time);

  valueDmm1.setLabel("Aktueller Wert DMM1");
  valueDmm1.setPrecision(6);
  valueDmm1.setColor("32DB44");
  valueDmm1.setChannel(1);
  viewTime.addElement(valueDmm1);

  valueDmm2.setLabel("Aktueller Wert DMM2");
  valueDmm2.setPrecision(6);
  valueDmm2.setColor("E4572E");
  valueDmm2.setChannel(2);
  viewTime.addElement(valueDmm2);

  phyphoxExperiment.addView(viewTime);

  PhyphoxBLE::addExperiment(phyphoxExperiment);
}

// =============================================================================
// Arduino setup
// =============================================================================

void setup() {
  initDmm(dmm1);
  initDmm(dmm2);

  setupPhyphoxExperiment();
  PhyphoxBLE::start(PHYPHOX_DEVICE_NAME);
}

// =============================================================================
// Arduino loop
// =============================================================================

void loop() {
  FS9721::Measurement measurement1;
  FS9721::Measurement measurement2;

  const bool got1 = pollDmm(dmm1, measurement1);
  const bool got2 = pollDmm(dmm2, measurement2);

  if (got1 && measurement1.valid && !isinf(measurement1.value)) {
    latestValue1 = measurement1.value;
    haveValue1 = true;
  }

  if (got2 && measurement2.valid && !isinf(measurement2.value)) {
    latestValue2 = measurement2.value;
    haveValue2 = true;
  }

  // Sobald beide DMMs mindestens einen gültigen Wert geliefert haben,
  // bei jeder neuen Messung das aktuelle Wertepaar senden.
  if ((got1 || got2) && haveValue1 && haveValue2) {
    PhyphoxBLE::write(latestValue1, latestValue2);
  }
}
