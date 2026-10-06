#include <Arduino.h>
#include <math.h>
#include <phyphoxBle.h>


// =============================================================================
// Konfiguration
// =============================================================================

static constexpr char PHYPHOX_DEVICE_NAME[] =
  "Tecpel DMM-8061";

static constexpr uint32_t DMM_BAUDRATE = 2400;
static constexpr uint8_t FRAME_LEN = 14;

static constexpr int DMM1_RX_PIN = 4;
static constexpr int DMM1_TX_PIN = 5;

static constexpr int DMM2_RX_PIN = 3;
static constexpr int DMM2_TX_PIN = 21;


// =============================================================================
// Datenstrukturen
// =============================================================================

struct DmmMeasurement {
  float value = NAN;
  String unit;
  bool valid = false;
};


struct DmmPort {
  HardwareSerial *serial;
  int rxPin;
  int txPin;

  uint8_t frame[FRAME_LEN];
  uint8_t framePos = 0;
  bool inFrame = false;

  DmmMeasurement latest;
};


// =============================================================================
// DMM Ports
// =============================================================================

DmmPort dmm1{
  &Serial1,
  DMM1_RX_PIN,
  DMM1_TX_PIN
};

HardwareSerial DmmSerial2(2);

DmmPort dmm2{
  &DmmSerial2,
  DMM2_RX_PIN,
  DMM2_TX_PIN
};


// =============================================================================
// phyphox Experiment
// =============================================================================

PhyphoxBleExperiment phyphoxExperiment;

// View 1: DMM1 vs DMM2
PhyphoxBleExperiment::View viewXY;
PhyphoxBleExperiment::Graph graphXY;
PhyphoxBleExperiment::Value valueX;
PhyphoxBleExperiment::Value valueY;

// View 2: Einzelne Kanäle über Zeit
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

  dmm.framePos = 0;
  dmm.inFrame = false;
  dmm.latest.valid = false;
}


// =============================================================================
// FS9721-Ziffer dekodieren
// =============================================================================

int parseDigit(uint8_t firstByte, uint8_t secondByte) {
  uint8_t digitByte =
    static_cast<uint8_t>(
      ((firstByte & 0x0F) << 4) | (secondByte & 0x0F));

  digitByte &= 0x7F;

  switch (digitByte) {
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


// =============================================================================
// Synchronisationsprüfung
// =============================================================================

bool packetSyncIsValid(const uint8_t *frame) {
  for (uint8_t i = 0; i < FRAME_LEN; i++) {
    if (((frame[i] >> 4) & 0x0F) != (i + 1)) {
      return false;
    }
  }
  return true;
}


// =============================================================================
// Einheit dekodieren
// =============================================================================

void decodeUnit(const uint8_t *frame, String &unit) {
  unit = "";

  const uint8_t b0 = frame[0] & 0x0F;
  const uint8_t b9 = frame[9] & 0x0F;
  const uint8_t b10 = frame[10] & 0x0F;
  const uint8_t b11 = frame[11] & 0x0F;
  const uint8_t b12 = frame[12] & 0x0F;

  if (b12 & 0x04) {
    unit = "V";
  } else if (b12 & 0x08) {
    unit = "A";
  } else if (b11 & 0x04) {
    unit = "Ohm";
  } else if (b11 & 0x08) {
    unit = "F";
  } else if (b12 & 0x02) {
    unit = "Hz";
  } else if (b10 & 0x04) {
    unit = "%";
  } else if (b9 & 0x01) {
    unit = "V";
  } else if (b10 & 0x01) {
    unit = "Cont";
  }

  if (b0 & 0x08) {
    unit = "AC " + unit;
  } else if (b0 & 0x04) {
    unit = "DC " + unit;
  }
}


// =============================================================================
// FS9721-Frame dekodieren
// =============================================================================

bool decodeDmmFrame(const uint8_t *frame, DmmMeasurement &result) {
  if (!packetSyncIsValid(frame)) {
    return false;
  }

  uint8_t digitBytes[4];
  int digits[4];

  for (uint8_t i = 0; i < 4; i++) {
    digitBytes[i] =
      static_cast<uint8_t>(
        ((frame[1 + i * 2] & 0x0F) << 4) | (frame[2 + i * 2] & 0x0F))
      & 0x7F;

    digits[i] = parseDigit(frame[1 + i * 2], frame[2 + i * 2]);
  }

  // Overload check
  if (digitBytes[0] == 0x00 && digitBytes[1] == 0x7D && digitBytes[2] == 0x68 && digitBytes[3] == 0x00) {
    result.value = INFINITY;
    decodeUnit(frame, result.unit);
    result.valid = true;
    return true;
  }

  for (uint8_t i = 0; i < 4; i++) {
    if (digits[i] < 0) {
      result.valid = false;
      return false;
    }
  }

  float value =
    digits[0] * 1000.0f + digits[1] * 100.0f + digits[2] * 10.0f + digits[3];

  // Dezimalpunkt
  if (frame[3] & 0x08) {
    value *= 0.001f;
  } else if (frame[5] & 0x08) {
    value *= 0.01f;
  } else if (frame[7] & 0x08) {
    value *= 0.1f;
  }

  // Vorzeichen
  if (frame[1] & 0x08) {
    value = -value;
  }

  // SI-Multiplikatoren
  const uint8_t b9 = frame[9] & 0x0F;
  const uint8_t b10 = frame[10] & 0x0F;

  if (b9 & 0x04) {
    value *= 1.0e-9f;
  } else if (b9 & 0x08) {
    value *= 1.0e-6f;
  } else if (b10 & 0x08) {
    value *= 1.0e-3f;
  } else if (b9 & 0x02) {
    value *= 1.0e3f;
  } else if (b10 & 0x02) {
    value *= 1.0e6f;
  }

  result.value = value;
  decodeUnit(frame, result.unit);
  result.valid = true;

  return true;
}


// =============================================================================
// DMM pollen
// =============================================================================

bool pollDmm(DmmPort &dmm, DmmMeasurement &result) {
  if (dmm.serial->available() < FRAME_LEN) {
    return false;
  }

  while (dmm.serial->available() > 0) {
    uint8_t byte = static_cast<uint8_t>(dmm.serial->read());

    if (!dmm.inFrame && ((byte >> 4) == 0x01)) {
      dmm.inFrame = true;
      dmm.framePos = 0;
    }

    if (!dmm.inFrame) {
      continue;
    }

    dmm.frame[dmm.framePos++] = byte;

    if (dmm.framePos == FRAME_LEN) {
      dmm.inFrame = false;
      dmm.framePos = 0;

      if (decodeDmmFrame(dmm.frame, dmm.latest)) {
        result = dmm.latest;
        return true;
      }
    }
  }

  return false;
}


// =============================================================================
// phyphox Experiment setup
// =============================================================================

void setupPhyphoxExperiment() {
  phyphoxExperiment.setTitle("Tecpel DMM-8061 Dual");
  phyphoxExperiment.setCategory("Multimeter");
  phyphoxExperiment.setDescription("Zwei Tecpel DMM-8061 Messwerte");
  phyphoxExperiment.setColor("32DB44");


  // =========================================================================
  // View 1: DMM1 vs DMM2 (XY-Plot)
  // =========================================================================

  viewXY.setLabel("XY-Plot");

  graphXY.setLabel("DMM1 vs DMM2");
  graphXY.setLabelX("DMM1");
  graphXY.setLabelY("DMM2");
  graphXY.setXPrecision(6);
  graphXY.setYPrecision(6);
  graphXY.setStyle(STYLE_DOTS);
  graphXY.setColor("32DB44");
  graphXY.setLinewidth(1.0f);
  graphXY.setMinX(0, LAYOUT_AUTO);
  graphXY.setMaxX(0, LAYOUT_AUTO);
  graphXY.setMinY(0, LAYOUT_AUTO);
  graphXY.setMaxY(0, LAYOUT_AUTO);
  graphXY.setChannel(1, 2);  // CH1 (DMM1) vs CH2 (DMM2)

  viewXY.addElement(graphXY);

  valueX.setLabel("Aktueller Wert DMM1 (X)");
  valueX.setPrecision(6);
  valueX.setColor("32DB44");
  valueX.setChannel(1);

  viewXY.addElement(valueX);

  valueY.setLabel("Aktueller Wert DMM2 (Y)");
  valueY.setPrecision(6);
  valueY.setColor("32DB44");
  valueY.setChannel(2);

  viewXY.addElement(valueY);

  phyphoxExperiment.addView(viewXY);


  // =========================================================================
  // View 2: Kanäle über Zeit
  // =========================================================================

  viewTime.setLabel("Zeit");

  // Graph 1: DMM1 über Zeit
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
  graphDmm1Time.setChannel(0, 1);  // CH0 (Zeit), CH1 (DMM1)

  viewTime.addElement(graphDmm1Time);

  // Graph 2: DMM2 über Zeit
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
  graphDmm2Time.setChannel(0, 2);  // CH0 (Zeit), CH2 (DMM2)

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
  Serial.begin(115200);
  delay(1000);

  initDmm(dmm1);
  initDmm(dmm2);

  setupPhyphoxExperiment();
  PhyphoxBLE::start(PHYPHOX_DEVICE_NAME);
}


// =============================================================================
// Arduino loop
// =============================================================================

void loop() {
  DmmMeasurement measurement1;
  DmmMeasurement measurement2;

  bool have1 = pollDmm(dmm1, measurement1);
  bool have2 = pollDmm(dmm2, measurement2);

  float value1 = 0.0f;
  float value2 = 0.0f;

  if (have1 && measurement1.valid && !isinf(measurement1.value)) {
    value1 = measurement1.value;
  }

  if (have2 && measurement2.valid && !isinf(measurement2.value)) {
    value2 = measurement2.value;
  }

  if (have1 || have2) {
    PhyphoxBLE::write(value1, value2);
  }

  PhyphoxBLE::poll();
  delay(5);
}