#include <Arduino.h>
#include <math.h>
#include <phyphoxBle.h>


// =============================================================================
// Konfiguration
// =============================================================================

// Name, der in der phyphox-Geräteliste erscheint.
static constexpr char PHYPHOX_DEVICE_NAME[] =
    "Tecpel DMM-8061 ESP32-C3";

// Baudrate des Tecpel DMM-8061.
static constexpr uint32_t DMM_BAUDRATE = 2400;

// FS9721-Telegrammlänge.
static constexpr uint8_t FRAME_LEN = 14;

// Pause zwischen zwei Telegrammen.
static constexpr unsigned long FRAME_GAP_US = 150000UL;


// -----------------------------------------------------------------------------
// DMM 1
// -----------------------------------------------------------------------------

static constexpr int DMM1_RX_PIN = 4;
static constexpr int DMM1_TX_PIN = 5;


// -----------------------------------------------------------------------------
// Zweites DMM
//
// Beim ESP32-C3 ist normalerweise nur ein zusätzlicher Hardware-UART neben
// USB-Serial sinnvoll nutzbar. Deshalb ist DMM2 standardmäßig deaktiviert.
//
// Für klassische ESP32-Boards kann USE_DMM2 auf true gesetzt werden.
// Die Pins müssen dann an die Hardware angepasst werden.
// -----------------------------------------------------------------------------

static constexpr bool USE_DMM2 = false;

#if USE_DMM2
static constexpr int DMM2_RX_PIN = 16;
static constexpr int DMM2_TX_PIN = 17;
#endif


// =============================================================================
// Datenstrukturen
// =============================================================================

struct DmmMeasurement {
    float value = NAN;

    // Einheit ohne SI-Präfix, weil value bereits in SI-Basiseinheiten
    // umgerechnet wurde.
    //
    // Beispiel:
    // DMM-Anzeige: 8 mV
    // value:       0.008
    // unit:        "V"
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
    unsigned long lastByteMicros = 0;

    DmmMeasurement latest;
};


// =============================================================================
// Serielle DMM-Ports
// =============================================================================

DmmPort dmm1 {
    &Serial1,
    DMM1_RX_PIN,
    DMM1_TX_PIN
};

#if USE_DMM2
HardwareSerial DmmSerial2(2);

DmmPort dmm2 {
    &DmmSerial2,
    DMM2_RX_PIN,
    DMM2_TX_PIN
};
#endif


// =============================================================================
// phyphox-Experiment
// =============================================================================
//
// Die phyphox-Library erzeugt daraus automatisch die Benutzeroberfläche.
//
// Kanalbelegung:
//   CH0 = von phyphox erzeugter Zeitstempel
//   CH1 = DMM1-Wert
//   CH2 = DMM2-Wert, falls aktiviert
//
// Die Phyphox-Library verwendet für setChannel() eine nullbasierte Kanalnummer.
// =============================================================================

PhyphoxBleExperiment phyphoxExperiment;

PhyphoxBleExperiment::View mainView;

PhyphoxBleExperiment::Graph measurementGraph;

PhyphoxBleExperiment::Value currentValueDmm1;

#if USE_DMM2
PhyphoxBleExperiment::Value currentValueDmm2;
#endif


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
    dmm.lastByteMicros = 0;
    dmm.latest.valid = false;
}


// =============================================================================
// FS9721-Ziffer dekodieren
// =============================================================================
//
// Der FS9721 überträgt jede Ziffer über zwei Bytes:
//
//   digit = ((byte_odd & 0x0f) << 4) | (byte_even & 0x0f)
//
// Das obere Nibble jedes Bytes ist ein Synchronisations-Nibble.
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

        default:
            return -1;
    }
}


// =============================================================================
// Synchronisationsprüfung
// =============================================================================
//
// Gültige High-Nibbles:
//
//   1 2 3 4 5 6 7 8 9 A B C D E
// =============================================================================

bool packetSyncIsValid(const uint8_t *frame)
{
    for (uint8_t i = 0; i < FRAME_LEN; i++) {
        const uint8_t expected = i + 1;
        const uint8_t actual = (frame[i] >> 4) & 0x0F;

        if (actual != expected) {
            return false;
        }
    }

    return true;
}


// =============================================================================
// Einheit und Modus dekodieren
// =============================================================================
//
// Der SI-Präfix wird absichtlich nicht in unit aufgenommen.
//
// Beispiel:
//
//   Messwert: 0.008
//   Einheit:  "V"
//
// statt:
//
//   Messwert: 8
//   Einheit:  "mV"
//
// Dadurch bleibt der Zahlenwert unabhängig von der Displaydarstellung in
// SI-Basiseinheiten.
// =============================================================================

void decodeUnit(const uint8_t *frame, String &unit)
{
    unit = "";

    const uint8_t b0  = frame[0]  & 0x0F;
    const uint8_t b9  = frame[9]  & 0x0F;
    const uint8_t b10 = frame[10] & 0x0F;
    const uint8_t b11 = frame[11] & 0x0F;
    const uint8_t b12 = frame[12] & 0x0F;

    // Einheit
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
    } else {
        unit = "";
    }

    // AC/DC voranstellen
    if (b0 & 0x08) {
        unit = "AC " + unit;
    } else if (b0 & 0x04) {
        unit = "DC " + unit;
    }
}


// =============================================================================
// FS9721-Frame dekodieren
// =============================================================================

bool decodeDmmFrame(
    const uint8_t *frame,
    DmmMeasurement &result
)
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
            );

        digitBytes[i] &= 0x7F;

        digits[i] = parseDigit(
            frame[1 + i * 2],
            frame[2 + i * 2]
        );
    }

    // Overload-Darstellung des FS9721.
    const bool overLimit =
        digitBytes[0] == 0x00 &&
        digitBytes[1] == 0x7D &&
        digitBytes[2] == 0x68 &&
        digitBytes[3] == 0x00;

    if (overLimit) {
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

    int integerValue =
        digits[0] * 1000 +
        digits[1] * 100 +
        digits[2] * 10 +
        digits[3];

    float value = static_cast<float>(integerValue);

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
    //
    // Byte 9:
    //   bit 3 = micro
    //   bit 2 = nano
    //   bit 1 = kilo
    //
    // Byte 10:
    //   bit 3 = milli
    //   bit 1 = mega

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
//
// Rückgabe:
//   true  = neuer vollständiger Messwert
//   false = noch kein neuer Messwert
//
// Diese Funktion gibt nichts am seriellen Monitor aus.
// =============================================================================

bool pollDmm(
    DmmPort &dmm,
    DmmMeasurement &result
)
{
    while (dmm.serial->available()) {
        const uint8_t byte =
            static_cast<uint8_t>(dmm.serial->read());

        const unsigned long now = micros();

        const bool newFrame =
            dmm.lastByteMicros == 0 ||
            static_cast<unsigned long>(
                now - dmm.lastByteMicros
            ) > FRAME_GAP_US;

        dmm.lastByteMicros = now;

        if (newFrame) {
            dmm.framePos = 0;

            // Ein gültiger Frame beginnt mit einem Byte 0x1x.
            dmm.inFrame = ((byte >> 4) == 0x01);
        }

        if (!dmm.inFrame) {
            continue;
        }

        if (dmm.framePos >= FRAME_LEN) {
            dmm.framePos = 0;
            dmm.inFrame = false;
            continue;
        }

        dmm.frame[dmm.framePos++] = byte;

        if (dmm.framePos == FRAME_LEN) {
            const bool decoded =
                decodeDmmFrame(
                    dmm.frame,
                    dmm.latest
                );

            dmm.framePos = 0;
            dmm.inFrame = false;

            if (decoded) {
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

void printMeasurement(
    const char *name,
    const DmmMeasurement &measurement
)
{
    Serial.print(name);
    Serial.print(": ");

    if (!measurement.valid) {
        Serial.println("ungültig");
        return;
    }

    if (isinf(measurement.value)) {
        Serial.print("O.L");
    } else {
        Serial.print(measurement.value, 6);
    }

    Serial.print(" ");
    Serial.println(measurement.unit);
}


// =============================================================================
// phyphox-Oberfläche erzeugen
// =============================================================================

void setupPhyphoxExperiment()
{
    phyphoxExperiment.setTitle(
        "Tecpel DMM-8061 Messwerte"
    );

    phyphoxExperiment.setCategory(
        "Multimeter"
    );

    phyphoxExperiment.setDescription(
        "Messwerte des Tecpel DMM-8061. "
        "Die Zahlenwerte werden in SI-Basiseinheiten übertragen."
    );

    phyphoxExperiment.setColor(
        "F6F6F6"
    );


    // -------------------------------------------------------------------------
    // Hauptansicht
    // -------------------------------------------------------------------------

    mainView.setLabel(
        "Messung"
    );


    // -------------------------------------------------------------------------
    // Graph
    // -------------------------------------------------------------------------
    //
    // CH0 = phyphox-Zeitstempel
    // CH1 = DMM1
    // CH2 = DMM2
    //
    // x = 0, y = 1 bedeutet:
    // Messwert über der von phyphox erzeugten Zeit.
    // -------------------------------------------------------------------------

    measurementGraph.setUnitX(
        "s"
    );

    measurementGraph.setLabelX(
        "Zeit"
    );

    measurementGraph.setLabelY(
        "Messwert"
    );

    measurementGraph.setXPrecision(
        4
    );

    measurementGraph.setYPrecision(
        6
    );

    measurementGraph.setStyle(
        STYLE_LINES
    );

    measurementGraph.setColor(
        "32DB44"
    );

    measurementGraph.setLinewidth(
        2.0f
    );

    // Automatische Achsenskalierung.
    measurementGraph.setMinY(
        0,
        LAYOUT_AUTO
    );

    measurementGraph.setMaxY(
        0,
        LAYOUT_AUTO
    );

    // CH0 ist die von phyphox erzeugte Empfangszeit.
    // CH1 ist DMM1.
    measurementGraph.setChannel(
        0,
        1
    );

#if USE_DMM2

    // Zweite Kurve im selben Graphen.
    PhyphoxBleExperiment::Graph::Subgraph dmm2Subgraph;

    dmm2Subgraph.setChannel(
        0,
        2
    );

    dmm2Subgraph.setStyle(
        STYLE_LINES
    );

    dmm2Subgraph.setColor(
        "E4572E"
    );

    dmm2Subgraph.setLinewidth(
        2.0f
    );

    measurementGraph.addSubgraph(
        dmm2Subgraph
    );

#endif

    mainView.addElement(
        measurementGraph
    );


    // -------------------------------------------------------------------------
    // Aktueller Wert DMM1
    // -------------------------------------------------------------------------

    currentValueDmm1.setLabel(
        "Aktueller Messwert DMM1"
    );

    currentValueDmm1.setPrecision(
        6
    );

    currentValueDmm1.setColor(
        "F6F6F6"
    );

    currentValueDmm1.setChannel(
        1
    );

    mainView.addElement(
        currentValueDmm1
    );


#if USE_DMM2

    // -------------------------------------------------------------------------
    // Aktueller Wert DMM2
    // -------------------------------------------------------------------------

    currentValueDmm2.setLabel(
        "Aktueller Messwert DMM2"
    );

    currentValueDmm2.setPrecision(
        6
    );

    currentValueDmm2.setColor(
        "E4572E"
    );

    currentValueDmm2.setChannel(
        2
    );

    mainView.addElement(
        currentValueDmm2
    );

#endif

    phyphoxExperiment.addView(
        mainView
    );

    PhyphoxBLE::addExperiment(
        phyphoxExperiment
    );
}


// =============================================================================
// Arduino setup
// =============================================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);

    initDmm(dmm1);

#if USE_DMM2
    initDmm(dmm2);
#endif

    // Der Name erscheint beim Scannen in phyphox.
    PhyphoxBLE::start(
        PHYPHOX_DEVICE_NAME
    );

    setupPhyphoxExperiment();

    Serial.println();
    Serial.println("Tecpel DMM-8061 reader ready");
    Serial.print("phyphox device name: ");
    Serial.println(PHYPHOX_DEVICE_NAME);
}


// =============================================================================
// Arduino loop
// =============================================================================

void loop()
{
    DmmMeasurement newMeasurement;


    // -------------------------------------------------------------------------
    // DMM1 lesen
    // -------------------------------------------------------------------------

    if (pollDmm(dmm1, newMeasurement)) {
        printMeasurement(
            "DMM1",
            newMeasurement
        );

        // O.L wird nicht an phyphox gesendet, da Infinity kein sinnvoller
        // Float-Wert für die phyphox-Anzeige ist.
        if (newMeasurement.valid &&
            !isinf(newMeasurement.value)) {

            PhyphoxBLE::write(
                newMeasurement.value
            );
        }
    }


#if USE_DMM2

    // -------------------------------------------------------------------------
    // DMM2 lesen
    // -------------------------------------------------------------------------

    if (pollDmm(dmm2, newMeasurement)) {
        printMeasurement(
            "DMM2",
            newMeasurement
        );

        if (newMeasurement.valid &&
            !isinf(newMeasurement.value)) {

            // Wenn zwei Werte übertragen werden, müssen beide Werte in
            // demselben write()-Aufruf stehen.
            //
            // Für eine vollständig synchrone Zwei-DMM-Anwendung sollten die
            // letzten Werte beider DMMs zwischengespeichert werden.
        }
    }

#endif

    // Für den ESP32 ist poll() normalerweise nicht erforderlich.
    // Für einige andere Boards, die von der Library unterstützt werden,
    // ist es notwendig. Auf dem ESP32 schadet der Aufruf nicht.
    PhyphoxBLE::poll();

    delay(5);
}