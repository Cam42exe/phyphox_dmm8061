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

static constexpr int DMM_RX_PIN = 4;
static constexpr int DMM_TX_PIN = 5;


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
// DMM Port
// =============================================================================

DmmPort dmm {
    &Serial1,
    DMM_RX_PIN,
    DMM_TX_PIN
};


// =============================================================================
// phyphox Experiment
// =============================================================================

PhyphoxBleExperiment phyphoxExperiment;
PhyphoxBleExperiment::View mainView;
PhyphoxBleExperiment::Graph measurementGraph;
PhyphoxBleExperiment::Value currentValue;


// =============================================================================
// DMM initialisieren
// =============================================================================

void initDmm(DmmPort &dmm)
{
    dmm.serial->begin(
        DMM_BAUDRATE,
        SERIAL_8N1,
        dmm.rxPin,
        dmm.txPin
    );

    dmm.framePos = 0;
    dmm.inFrame = false;
    dmm.latest.valid = false;
}


// =============================================================================
// FS9721-Ziffer dekodieren
// =============================================================================

int parseDigit(uint8_t firstByte, uint8_t secondByte)
{
    uint8_t digitByte =
        static_cast<uint8_t>(
            ((firstByte & 0x0F) << 4) |
             (secondByte & 0x0F)
        );

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

bool packetSyncIsValid(const uint8_t *frame)
{
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

void decodeUnit(const uint8_t *frame, String &unit)
{
    unit = "";

    const uint8_t b0  = frame[0]  & 0x0F;
    const uint8_t b9  = frame[9]  & 0x0F;
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

bool decodeDmmFrame(const uint8_t *frame, DmmMeasurement &result)
{
    if (!packetSyncIsValid(frame)) {
        return false;
    }

    uint8_t digitBytes[4];
    int digits[4];

    for (uint8_t i = 0; i < 4; i++) {
        digitBytes[i] =
            static_cast<uint8_t>(
                ((frame[1 + i * 2] & 0x0F) << 4) |
                 (frame[2 + i * 2] & 0x0F)
            ) & 0x7F;

        digits[i] = parseDigit(frame[1 + i * 2], frame[2 + i * 2]);
    }

    // Overload check
    if (digitBytes[0] == 0x00 && digitBytes[1] == 0x7D &&
        digitBytes[2] == 0x68 && digitBytes[3] == 0x00) {
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
        digits[0] * 1000.0f +
        digits[1] * 100.0f +
        digits[2] * 10.0f +
        digits[3];

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
    const uint8_t b9  = frame[9]  & 0x0F;
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

bool pollDmm(DmmPort &dmm, DmmMeasurement &result)
{
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
// Serielle Ausgabe
// =============================================================================

void printMeasurement(const DmmMeasurement &measurement)
{
    if (!measurement.valid) {
        Serial.println("ungültig");
        return;
    }

    if (isinf(measurement.value)) {
        Serial.println("O.L");
    } else {
        Serial.print(measurement.value, 6);
        Serial.print(" ");
        Serial.println(measurement.unit);
    }
}


// =============================================================================
// phyphox Experiment setup
// =============================================================================

void setupPhyphoxExperiment()
{
    phyphoxExperiment.setTitle("Tecpel DMM-8061");
    phyphoxExperiment.setCategory("Multimeter");
    phyphoxExperiment.setDescription("Messwerte des Tecpel DMM-8061");
    phyphoxExperiment.setColor("32DB44");

    mainView.setLabel("Messung");

    // Graph: Zeit auf X-Achse (CH0), Messwert auf Y-Achse (CH1)
    measurementGraph.setLabel("Messwert über Zeit");
    measurementGraph.setLabelX("Zeit");
    measurementGraph.setLabelY("Messwert");
    measurementGraph.setXPrecision(1);
    measurementGraph.setYPrecision(6);
    measurementGraph.setStyle(STYLE_LINES);
    measurementGraph.setColor("32DB44");
    measurementGraph.setLinewidth(2.0f);
    measurementGraph.setMinY(0, LAYOUT_AUTO);
    measurementGraph.setMaxY(0, LAYOUT_AUTO);
    measurementGraph.setChannel(0, 1);  // CH0 Zeit, CH1 Messwert

    mainView.addElement(measurementGraph);

    // Aktueller Wert
    currentValue.setLabel("Aktueller Messwert");
    currentValue.setPrecision(6);
    currentValue.setColor("32DB44");
    currentValue.setChannel(1);

    mainView.addElement(currentValue);

    phyphoxExperiment.addView(mainView);
    PhyphoxBLE::addExperiment(phyphoxExperiment);
}


// =============================================================================
// Arduino setup
// =============================================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    initDmm(dmm);

    setupPhyphoxExperiment();
    PhyphoxBLE::start(PHYPHOX_DEVICE_NAME);

    Serial.println();
    Serial.println("Tecpel DMM-8061 reader ready");
    Serial.print("Device name: ");
    Serial.println(PHYPHOX_DEVICE_NAME);
}


// =============================================================================
// Arduino loop
// =============================================================================

void loop()
{
    DmmMeasurement measurement;

    if (pollDmm(dmm, measurement)) {
        Serial.print("Messung: ");
        printMeasurement(measurement);

        if (measurement.valid && !isinf(measurement.value)) {
            PhyphoxBLE::write(measurement.value);
        }
    }

    PhyphoxBLE::poll();
    delay(5);
}