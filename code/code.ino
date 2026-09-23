#include <Arduino.h>
#include <math.h>

static constexpr uint8_t FRAME_LEN = 14;
static constexpr unsigned long FRAME_GAP_US = 150000UL;


// -----------------------------------------------------------------------------
// Ergebnis einer Messung
// -----------------------------------------------------------------------------

struct DmmMeasurement {
    float value = NAN;
    String unit;
    bool valid = false;
};


// -----------------------------------------------------------------------------
// Zustand eines DMM-Eingangs
// -----------------------------------------------------------------------------

struct DmmPort {
    HardwareSerial *serial;
    int rxPin;
    int txPin;

    uint8_t frame[FRAME_LEN];
    uint8_t framePos = 0;

    bool inFrame = false;
    unsigned long lastByteMicros = 0;

    DmmMeasurement measurement;
};


// -----------------------------------------------------------------------------
// DMM-Eingänge konfigurieren
// -----------------------------------------------------------------------------

DmmPort dmm1 {
    &Serial1,
    4,       // RX
    5        // TX, wird beim reinen Empfang normalerweise nicht benötigt
};

#if defined(CONFIG_IDF_TARGET_ESP32) || \
    defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32S3)

DmmPort dmm2 {
    &Serial2,
    16,      // Beispiel-Pin, an deine Hardware anpassen
    17
};

#endif


// -----------------------------------------------------------------------------
// DMM-Serialport initialisieren
// -----------------------------------------------------------------------------

void initDmm(DmmPort &dmm)
{
    dmm.serial->begin(
        2400,
        SERIAL_8N1,
        dmm.rxPin,
        dmm.txPin
    );

    dmm.framePos = 0;
    dmm.inFrame = false;
    dmm.lastByteMicros = 0;
    dmm.measurement.valid = false;
}


// -----------------------------------------------------------------------------
// FS9721-Ziffer dekodieren
// -----------------------------------------------------------------------------

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


// -----------------------------------------------------------------------------
// FS9721-Synchronisation prüfen
//
// Gültige High-Nibbles:
//
//   1 2 3 4 5 6 7 8 9 A B C D E
// -----------------------------------------------------------------------------

bool packetSyncIsValid(const uint8_t *frame)
{
    for (uint8_t i = 0; i < FRAME_LEN; i++) {
        uint8_t expected = i + 1;
        uint8_t actual = (frame[i] >> 4) & 0x0F;

        if (actual != expected) {
            return false;
        }
    }

    return true;
}


// -----------------------------------------------------------------------------
// Einheit dekodieren
//
// Der Zahlenwert wird später bereits mit dem SI-Faktor multipliziert.
// Deshalb wird der Präfix hier absichtlich nicht zur Ausgabe-Einheit
// hinzugefügt.
//
// Beispiel:
//   Anzeige: 8 mV
//   Ergebnis: value = 0.008
//   unit    = "V"
// -----------------------------------------------------------------------------

void decodeUnit(const uint8_t *frame, String &unit)
{
    unit = "";

    uint8_t b9  = frame[9]  & 0x0F;
    uint8_t b10 = frame[10] & 0x0F;
    uint8_t b11 = frame[11] & 0x0F;
    uint8_t b12 = frame[12] & 0x0F;

    // Basiseinheit
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

    // AC/DC voranstellen
    uint8_t b0 = frame[0] & 0x0F;

    if (b0 & 0x08) {
        unit = "AC " + unit;
    } else if (b0 & 0x04) {
        unit = "DC " + unit;
    }
}


// -----------------------------------------------------------------------------
// Messwert eines FS9721-Frames dekodieren
// -----------------------------------------------------------------------------

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
        digits[i] = parseDigit(frame[1 + i * 2],
                               frame[2 + i * 2]);
    }

    // Overload-Darstellung laut sigrok
    bool overLimit =
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

    // SI-Multiplikatoren anwenden.
    //
    // Byte 9:
    //   bit 3 = micro
    //   bit 2 = nano
    //   bit 1 = kilo
    //
    // Byte 10:
    //   bit 3 = milli
    //   bit 1 = mega
    //
    uint8_t b9  = frame[9]  & 0x0F;
    uint8_t b10 = frame[10] & 0x0F;

    if (b9 & 0x04) {
        value *= 1.0e-9f;       // nano
    } else if (b9 & 0x08) {
        value *= 1.0e-6f;       // micro
    } else if (b10 & 0x08) {
        value *= 1.0e-3f;       // milli
    } else if (b9 & 0x02) {
        value *= 1.0e3f;        // kilo
    } else if (b10 & 0x02) {
        value *= 1.0e6f;        // mega
    }

    result.value = value;
    decodeUnit(frame, result.unit);
    result.valid = true;

    return true;
}


// -----------------------------------------------------------------------------
// DMM lesen
//
// Rückgabe:
//   true  = ein vollständiger neuer Messwert wurde empfangen
//   false = noch kein vollständiger Messwert vorhanden
//
// Der Wert selbst steht in:
//   result.value
//
// Die Einheit steht in:
//   result.unit
// -----------------------------------------------------------------------------

bool pollDmm(
    DmmPort &dmm,
    DmmMeasurement &result
)
{
    while (dmm.serial->available()) {
        uint8_t b =
            static_cast<uint8_t>(dmm.serial->read());

        unsigned long now = micros();

        bool newFrame =
            dmm.lastByteMicros == 0 ||
            static_cast<unsigned long>(
                now - dmm.lastByteMicros
            ) > FRAME_GAP_US;

        dmm.lastByteMicros = now;

        if (newFrame) {
            dmm.framePos = 0;

            // Ein Frame muss mit einem Byte 0x1x beginnen.
            dmm.inFrame = ((b >> 4) == 0x01);
        }

        if (!dmm.inFrame) {
            continue;
        }

        if (dmm.framePos >= FRAME_LEN) {
            dmm.framePos = 0;
            dmm.inFrame = false;
            continue;
        }

        dmm.frame[dmm.framePos++] = b;

        if (dmm.framePos == FRAME_LEN) {
            bool decoded =
                decodeDmmFrame(dmm.frame, dmm.measurement);

            dmm.framePos = 0;
            dmm.inFrame = false;

            if (decoded) {
                result = dmm.measurement;
                return true;
            }
        }
    }

    return false;
}


// -----------------------------------------------------------------------------
// Beispielausgabe im Hauptprogramm
// -----------------------------------------------------------------------------

void printMeasurement(
    const char *name,
    const DmmMeasurement &measurement
)
{
    Serial.print(name);
    Serial.print(": ");

    if (isinf(measurement.value)) {
        Serial.print("O.L");
    } else {
        Serial.print(measurement.value, 6);
    }

    Serial.print(" ");
    Serial.println(measurement.unit);
}


// -----------------------------------------------------------------------------
// Arduino setup
// -----------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(1000);

    initDmm(dmm1);

#if defined(CONFIG_IDF_TARGET_ESP32) || \
    defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32S3)

    initDmm(dmm2);
#endif

    Serial.println("DMM reader ready");
}


// -----------------------------------------------------------------------------
// Arduino loop
// -----------------------------------------------------------------------------

void loop()
{
    DmmMeasurement measurement;

    // DMM 1 auslesen
    if (pollDmm(dmm1, measurement)) {
        printMeasurement("DMM1", measurement);
    }

#if defined(CONFIG_IDF_TARGET_ESP32) || \
    defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32S3)

    // DMM 2 auslesen
    if (pollDmm(dmm2, measurement)) {
        printMeasurement("DMM2", measurement);
    }
#endif
}