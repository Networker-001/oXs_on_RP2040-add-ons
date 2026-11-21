#include "inav_frsky.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "ws2812.h"
#include <math.h> // Ermöglicht atan2f und sqrtf 

// ====================================================================
// UNIVERSAL PRIORITY POLL-SCHEDULER (FrSky S.Port Master-Mode)
// ====================================================================
// Dieses System simuliert einen universellen FrSky-Empfänger nach dem
// im Netz etablierten "Interleaved Group Scheduler"-Prinzip (Non-Blocking).
//
// TAKTUNG: Alle 12ms wird stufenweise ein neuer Sensor angepollt.
// DYNAMIK: Wichtige Telemetriedaten werden bevorzugt, während der Bus
//          permanent und flexibel nach neuen/veränderten Sensoren sucht.
//
// 📊 DIE ABFRAGE-REIHENFOLGE (SCHRITT-FÜR-SCHRITT):
// --------------------------------------------------------------------
// Takt | Stufe  | Klasse      | Ziel-Sensor / Funktion
// --------------------------------------------------------------------
//  0ms | Case 0 | Klasse A    | Echtzeit: Vario (0x00) oder iNav FC (0x1B)
// 12ms | Case 1 | Klasse B    | Fließend: Strom / Volt / UniSens-E (0x02)
// 24ms | Case 2 | Klasse A    | Echtzeit: Vario (0x00) oder iNav FC (0x1B) [DOPPELTER TAKT!]
// 36ms | Case 3 | Klasse C    | Suche: Rollierender "Suchscheinwerfer" (0x00 bis 0x1B)
// 48ms | Case 0 | Klasse A    | Loop beginnt von vorne...
// --------------------------------------------------------------------
//
// 🎯 VISUELLE DARSTELLUNG DES DAUERELOODS (Beispiel iNav + UniSens):
//
// Runde 1:  [0x00 (Vario)]  ->  [0x02 (Strom)]  ->  [0x1B (iNav)]  ->  [0x00 (Suche ID 0)]
// Runde 2:  [0x00 (Vario)]  ->  [0x02 (Strom)]  ->  [0x1B (iNav)]  ->  [0x01 (Suche ID 1)]
// Runde 3:  [0x00 (Vario)]  ->  [0x02 (Strom)]  ->  [0x1B (iNav)]  ->  [0x02 (Suche ID 2)]
// ...
// Runde 28: [0x00 (Vario)]  ->  [0x02 (Strom)]  ->  [0x1B (iNav)]  ->  [0x1B (Suche ID 27)]
//
// PROFI-VORTEILE DIESES KONZEPTS:
// 1. Klasse A (Vario & iNav) erhält die exakt doppelte Abfrage-Frequenz.
//    Das garantiert ein absolut verzögerungsfreies Vario-Piepsen am Sender.
// 2. Klasse C schläft nie. Der "Suchscheinwerfer" klappert permanent alle
//    28 IDs ab. Sensoren können mitten im Betrieb ab- oder angesteckt 
//    werden und werden innerhalb von maximal 336ms vollautomatisch erkannt.
// ====================================================================



//=====================================================================


// SW Version

int SW_Version = 55;

//=====================================================================





// 🟢 DIE OFFIZIELLE FR-SKY TELEMETRIE-LISTE (Nach Pawelsky)
static const uint8_t s_port_search_ids[] = {
    0x00, // ID1:  Variometer (Höhe / Steigen - Hauptadresse)
    0xA1, // ID2:  FLVS (Lipo-Einzelzellen-Spannungssensor)
    0x22, // ID3:  FAS (Klassischer Strom- und Spannungssensor)
    0x83, // ID4:  GPS-Empfänger (Breitengrad, Längengrad, Satelliten)
    0xE4, // ID5:  RPM / TEMPERATUR (Drehzahl und Motortemperatur)
    0x45, // ID6:  Airspeed (Staudruck-Geschwindigkeitsmesser)
    0xC6, // ID7:  Variometer (Alternative Zweitadresse)
    0x67, // ID8:  Sensor Hub (Verteilerbox für Zusatzsensoren)
    0x48, // ID9:  Frei / Universeller Zusatzsensor 1
    0xE9, // ID10: Frei / Universeller Zusatzsensor 2
    0x6A, // ID11: Frei / Universeller Zusatzsensor 3
    0xCB, // ID12: Frei / Universeller Zusatzsensor 4
    0xAC, // ID13: Frei / Universeller Zusatzsensor 5
    0x0D, // ID14: Frei / Universeller Zusatzsensor 6
    0x8E, // ID15: Frei / Universeller Zusatzsensor 7
    0x2F, // ID16: UniSens-E Standard-Adresse (Volt, Strom, Kapazität, RPM)
    0xD0, // ID17: Frei / Universeller Zusatzsensor 8
    0x71, // ID18: Frei / Universeller Zusatzsensor 9
    0xF2, // ID19: Frei / Universeller Zusatzsensor 10
    0x53, // ID20: Redundanz-Empfänger / Wechselschalter (RB-20/40 Block 1)
    0x34, // ID21: Redundanz-Empfänger / Wechselschalter (RB-20/40 Block 2)
    0x95, // ID22: Frei / Universeller Zusatzsensor 11
    0x16, // ID23: Frei / Universeller Zusatzsensor 12
    0xB7, // ID24: Frei / Universeller Zusatzsensor 13
    0x98, // ID25: Frei / Universeller Zusatzsensor 14
    0x39, // ID26: Frei / Universeller Zusatzsensor 15
    0xBA, // ID27: Frei / Universeller Zusatzsensor 16
    0x1B  // ID28: iNav Flight Controller (FCS kombiniert / Spannung, Strom, GPS)
};



//#define FRSKY_PIN1 9  

//#define DEBUGINAV
//#define DEBUGINAV3
extern uint8_t debugTlm;
extern uint8_t debugVal;
extern uint8_t debugAll;
extern uint8_t debugFrS;

// 🟢 KORREKTUR: Wir nutzen den echten oXs-Typ FIELD (Großbuchstaben)
// Damit weiß der Compiler exakt, wie das oXs-Array aufgebaut ist.
extern field fields[]; 
int8_t FRSKY_PIN1 = 2; 
extern CONFIG config; 

extern bool ltm_data_rx_flag;

// Globales Lipo-Array für die Einzelzellen (Einheit: Millivolt)
volatile uint16_t rx_cells_mvolt[8] = {0, 0, 0, 0, 0, 0, 0, 0};

bool has_rx_cells = false;


static int search_idx = 0;       
static int scheduler_step = 0;    
static uint32_t next_send_time_us = 0;
static uint32_t pin_listen_time_us = 0;
static bool pin_is_listening = false;

// Dynamische Registry für den Turbo-Takt
#define MAX_ACTIVE_SENSORS 10
static uint8_t active_sensors[MAX_ACTIVE_SENSORS];       
static uint32_t last_seen_ms[MAX_ACTIVE_SENSORS];        
static int active_sensors_count = 0;                     
static int class_b_rotation_idx = 0;                     

// Lokaler Zwischenspeicher für Empfangsbytes
static uint8_t local_rx_buf[30];
static int local_rx_idx = 0;

// Globale Variablen für den Zustand des Index-Schalters
bool istIndiziert = false;
uint8_t aktiverIndex = 0;


// Globale Zwischenspeicher NUR für das saubere Debug-Printing
#ifdef DEBUGINAV
static float dbg_volt = 0.0f;
static float dbg_strom = 0.0f;
static float dbg_hoehe = 0.0f;
static int32_t dbg_vario = 0;
static int32_t dbg_kap = 0;
static uint32_t dbg_last_print_ms = 0;
static int32_t dbg_rpm = 0; 
#endif


static int get_sensor_registry_index(uint8_t id) {
    for (int i = 0; i < active_sensors_count; i++) {
        if (active_sensors[i] == id) return i;
    }
    return -1;
}

static void unregister_sensor(int index) {
    if (index < 0 || index >= active_sensors_count) return;
    for (int i = index; i < active_sensors_count - 1; i++) {
        active_sensors[i] = active_sensors[i + 1];
        last_seen_ms[i] = last_seen_ms[i + 1];
    }
    active_sensors_count--;
    if (class_b_rotation_idx >= active_sensors_count) {
        class_b_rotation_idx = 0;
    }
}

static void send_software_uart_byte(uint8_t byte) {
    const uint64_t bit_time_us = 17; 
    uint64_t next_bit_time = time_us_64();
    gpio_put(FRSKY_PIN1, 1);
    next_bit_time += bit_time_us;
    while (time_us_64() < next_bit_time); 
    for (int i = 0; i < 8; i++) {
        if (byte & (1 << i)) gpio_put(FRSKY_PIN1, 0); 
        else gpio_put(FRSKY_PIN1, 1); 
        next_bit_time += bit_time_us;
        while (time_us_64() < next_bit_time);
    }
    gpio_put(FRSKY_PIN1, 0);
    next_bit_time += bit_time_us; 
    while (time_us_64() < next_bit_time);
}

static bool receive_software_uart_byte(uint8_t *received_byte) {
    if (gpio_get(FRSKY_PIN1) == 1) { 
        const uint32_t bit_time_us = 17;
        uint32_t next_sample_time = time_us_32();
        next_sample_time += (bit_time_us / 2);
        while (time_us_32() < next_sample_time);
        if (gpio_get(FRSKY_PIN1) == 0) return false;
        uint8_t b = 0;
        for (int i = 0; i < 8; i++) {
            next_sample_time += bit_time_us;
            while (time_us_32() < next_sample_time);
            if (gpio_get(FRSKY_PIN1) == 0) { b |= (1 << i); }
        }
        next_sample_time += bit_time_us;
        while (time_us_32() < next_sample_time);
        *received_byte = b;
        return true;
    }
    return false;
}

// DA=Y
void printAllTelemetryValues_0() {
    if (debugAll != 'Y') return;

    // Aufruf auf 2 Hz (1000 ms) einbremsen
    static uint32_t lastPrintTime = 0;
    uint32_t currentTime = to_ms_since_boot(get_absolute_time()); 
    if (currentTime - lastPrintTime < 500) return; 
    lastPrintTime = currentTime; 

    static int lastTotalLines = 0;
    int coolTermWindowHeight = 45; // Passe dies an deine CoolTerm-Fensterhöhe an

    if (lastTotalLines > 0) {
        int paddingLines = coolTermWindowHeight - lastTotalLines;
        if (paddingLines < 1) paddingLines = 1;
        for (int i = 0; i < paddingLines; i++) {
            printf("\n");
        }
    }

    int currentLinesCount = 0;

    // Kopfzeile drucken
    printf("=== oXs RP2040 Telemetry Live-Monitor (2 Hz) ===\n");
    printf("--------------------------------------------------------------------------------------\n");
    currentLinesCount += 2;

  
    // Matrix-Definition mit angepasster Reihenfolge (Höhen nach oben gerückt)
  
    const int telemetryMatrix[][3] = {
        { MVOLT, CURRENT, CAPACITY },                               // Zeile 1: Antrieb & Strom
        { RELATIVEALT, VSPEED, RESERVE1 },                    // Zeile 2: RELATIVEALT an 1. Stelle, VSPEED an 2. Stelle
        { ALTITUDE, LATITUDE, LONGITUDE},                          // Zeile 3: GPS Core
        { GROUNDSPEED, HEADING, NUMSAT },                           // Zeile 4: GPS Bewegung
        { GPS_DATE, GPS_TIME, GPS_PDOP },                           // Zeile 5: GPS Zeit & Signal
        { GPS_HOME_BEARING, GPS_HOME_DISTANCE, GPS_CUMUL_DIST },    // Zeile 6: GPS Home & Distanz
        { TEMP1, TEMP2, NUMBER_MAX_IDX },                           // Zeile 7: Temperaturen
        { PITCH, ROLL, YAW },                                       // Zeile 8: IMU Lage
        { ACC_X, ACC_Y, ACC_Z },                                    // Zeile 9: IMU Beschleunigung
        { RPM, AIRSPEED, AIRSPEED_COMPENSATED_VSPEED },             // Zeile 10: Drehzahl & Airspeed
        { ADS_1_1, ADS_1_2, ADS_1_3 },                              // Zeile 11: ADS Wandler 1
        { ADS_1_4, ADS_2_1, ADS_2_2 },                              // Zeile 12: ADS Wandler 2
        { ADS_2_3, ADS_2_4, NUMBER_MAX_IDX },                       // Zeile 13: ADS Wandler 3
        { SBUS_HOLD_COUNTER, SBUS_FAILSAFE_COUNTER, RESERVE3 },     // Zeile 14: RC-Signal & Status
        { RESERVE4, RESERVE5, NUMBER_MAX_IDX }                      // Zeile 15: Sequencer & AoA
    };



    const int totalRows = sizeof(telemetryMatrix) / sizeof(telemetryMatrix[0]);

    // Array mit den Klarnamen der Felder (starr gekürzt auf max. 12 Zeichen)
    const char* fieldNames[] = {
        "LATITUDE", "LONGITUDE", "GROUNDSPEED", "HEADING", "ALTITUDE",
        "NUMSAT", "GPS_DATE", "GPS_TIME", "GPS_PDOP", "GPS_HOME_BEA",
        "GPS_HOME_DIS", "MVOLT", "CURRENT", "RESERVE1", "RESERVE2",
        "CAPACITY", "TEMP1", "TEMP2", "VSPEED", "RELATIVEALT",
        "PITCH", "ROLL", "YAW", "RPM", "ADS_1_1",
        "ADS_1_2", "ADS_1_3", "ADS_1_4", "ADS_2_1", "ADS_2_2",
        "ADS_2_3", "ADS_2_4", "AIRSPEED", "AIRSPEED_COM", "SBUS_HOLD",
        "SBUS_FAILSAFE", "GPS_CUMUL_DI", "ACC_X", "ACC_Y", "ACC_Z",
        "RESERVE3", "RESERVE4", "RESERVE5", "RESERVE6", "RESERVE7"
    };

    // Das überarbeitete, intelligente Formatierungs-Lambda
    auto formatField = [](char* dest, int id, const char* nameStr) {
        if (id == NUMBER_MAX_IDX || nameStr[0] == '-') {
            // Vollkommen leeres Spaltenfenster (Breite: 25 Zeichen) für ausgeblendete Platzhalter
            snprintf(dest, 27, "%-25s", "");
            return;
        }

        // Falls Sensor komplett inaktiv ist -> Namen linksbündig, "n/a" rechtsbündig
        if (!fields[id].available && !fields[id].onceAvailable) {
            snprintf(dest, 27, "%-13s %11s", nameStr, "n/a");
            return;
        }

        int32_t rawVal = (int32_t)fields[id].value;
        char valBuffer[16];

        // Unterscheidung der Kommastellen je nach Sensor-ID
        if (id == LATITUDE || id == LONGITUDE) {
            // 7 Nachkommastellen (oXs GPS Format)
            int32_t deg = rawVal / 10000000;
            int32_t frac = abs(rawVal % 10000000);
            snprintf(valBuffer, sizeof(valBuffer), "%ld.%07ld", deg, frac);
        }
        else if (id == MVOLT || id == CURRENT || id == RESERVE1 || (id >= ADS_1_1 && id <= ADS_2_4)) {
            // 3 Nachkommastellen (Volt, Ampere, ADS)
            int32_t mainUnit = rawVal / 1000;
            int32_t frac = abs(rawVal % 1000);
            snprintf(valBuffer, sizeof(valBuffer), "%ld.%03ld", mainUnit, frac);
        }
       /* else if (id == ALTITUDE || id == VSPEED || id == HEADING) {                 //  || id == RELATIVEALT 
            // 2 Nachkommastellen (Meter aus cm, m/s aus cm/s, Grad aus 0.01 Grad)
            int32_t mainUnit = rawVal / 10000000; // oXs nutzt intern für Alt oft Skalierung, hier angepasst auf Standard-cm
            // Hinweis: Falls deine oXs Version cm liefert, hier durch 100 teilen:
            mainUnit = rawVal / 100;
            int32_t frac = abs(rawVal % 100);
            snprintf(valBuffer, sizeof(valBuffer), "%ld.%02ld", mainUnit, frac);
        }*/
        else if (id == ALTITUDE || id == RELATIVEALT || id == VSPEED || id == HEADING) {                 
            // Hinweis: Falls deine oXs Version cm liefert, hier durch 100 teilen:
            int32_t mainUnit = rawVal / 100;
            int32_t frac = abs(rawVal % 100);
    
            // Prüfen, ob der Wert negativ ist (auch wenn mainUnit zu 0 wird)
            if (rawVal < 0) {
                snprintf(valBuffer, sizeof(valBuffer), "-%ld.%02ld", abs(mainUnit), frac);
            } else {
                snprintf(valBuffer, sizeof(valBuffer), "%ld.%02ld", mainUnit, frac);
            }
        }

        else if (id == GPS_DATE) {
            // Formatierung TT.MM.JJ
            int32_t day = rawVal / 1000000;
            int32_t month = (rawVal / 10000) % 100;
            int32_t year = (rawVal / 100) % 100;
            snprintf(valBuffer, sizeof(valBuffer), "%02ld.%02ld.%02ld", day, month, year);
        }
        else if (id == GPS_TIME) {
            // Formatierung HH:MM:SS
            int32_t hour = rawVal / 1000000;
            int32_t min = (rawVal / 10000) % 100;
            int32_t sec = (rawVal / 100) % 100;
            snprintf(valBuffer, sizeof(valBuffer), "%02ld:%02ld:%02ld", hour, min, sec);
        }
        else {
            // Reine Ganzzahlen (NUMSAT, CAPACITY, RPM, Counter etc.)
            snprintf(valBuffer, sizeof(valBuffer), "%ld", rawVal);
        }

        // Zusammenbauen: Name starr linksbündig (13 Zeichen), Wert starr rechtsbündig (11 Zeichen)
        snprintf(dest, 27, "%-13s %11s", nameStr, valBuffer);
    };

    // Matrix ausgeben
    for (int r = 0; r < totalRows; r++) {
        int id1 = telemetryMatrix[r][0];
        int id2 = telemetryMatrix[r][1];
        int id3 = telemetryMatrix[r][2];

        bool anyAvailable = false;
        if (id1 != NUMBER_MAX_IDX && (fields[id1].available || fields[id1].onceAvailable)) anyAvailable = true;
        if (id2 != NUMBER_MAX_IDX && (fields[id2].available || fields[id2].onceAvailable)) anyAvailable = true;
        if (id3 != NUMBER_MAX_IDX && (fields[id3].available || fields[id3].onceAvailable)) anyAvailable = true;

        if (!anyAvailable) continue;

        char col1[32], col2[32], col3[32];

        formatField(col1, id1, (id1 == NUMBER_MAX_IDX) ? "---" : fieldNames[id1]);
        formatField(col2, id2, (id2 == NUMBER_MAX_IDX) ? "---" : fieldNames[id2]);
        formatField(col3, id3, (id3 == NUMBER_MAX_IDX) ? "---" : fieldNames[id3]);

        printf(" %s | %s | %s\n", col1, col2, col3);
        currentLinesCount++;
    }

    // Fußzeile drucken
    printf("-------------------------- stopp mit: DA = N -------------------------------------\n");
    currentLinesCount++;

    lastTotalLines = currentLinesCount;
    fflush(stdout); 
}



#include "hardware/sync.h" // Wichtig für hardware_spinlock

#define MIN_INTERVAL_MS 10  // Gewünschte Mindestpause zwischen Updates eines Feldes


// Das neue Makro nutzt den Pointer-Trick, um den Index im fields-Array zu bestimmen
#define setField(f, v) do { \
    sent2Core0((&(f) - fields), (int32_t)(v)); \
    if (debugVal == 'Y') { \
        printf("\r %-25s | Value: %8ld        ", #f, (int32_t)(v)); \
        fflush(stdout); \
    } \
} while(0)



// INDEX Überprüfen
uint16_t filterFrSkyId_o(uint16_t originaleId) {
/**
 * Verarbeitet die FrSky-ID und filtert/bereinigt sie basierend auf dem erkannten Index.
 * 
 * @param originaleId Die unveränderte ID aus dem Datenpaket
 * @return Die bereinigte ID für das Switch-Case, oder 0 wenn das Paket verworfen werden muss.
 */

 
    // GPS und andere IDs ab 0x0800 werden ungefiltert durchgewinkt
    if ((originaleId >= 0x0800 && (originaleId & 0xFFF0) != 0x0880 && 
        (originaleId & 0xFFF0) != 0x0B60 && (originaleId & 0xFFF0) != 0x0A10  )|| (originaleId & 0xFFF0) == 0x0300 ){
        return originaleId;
    }

    uint8_t letzteStelle = originaleId & 0x000F;
    uint16_t basisId = originaleId & 0xFFF0;

    // --- SCHICHT 1: INDEX-ERKENNUNG (Nur beim allerersten indizierten Paket) ---
    if (!istIndiziert) {
        // Ausnahme-Gruppe: 0x0210 (Spannung/Kapazität)
        /*if (basisId == 0x0510) {
            if (letzteStelle > 1) {
                istIndiziert = true;
                aktiverIndex = letzteStelle - 1; // z.B. 0x0513 -> Index 2
            }
        } */
        // Standard-Gruppen: 0x0100 (Höhe), 0x0110 (Vario), 0x0200 (Strom), 0x0210 (MVOLT)
        if (basisId == 0x0100 || basisId == 0x0110 || basisId == 0x0200 || basisId == 0x0210) {
            if (letzteStelle != 0) {
                istIndiziert = true;
                aktiverIndex = letzteStelle; // z.B. 0x0102 -> Index 2
            }
        }
        // Sicherheitsnetz für alle anderen IDs unter 0x0800
        else if (letzteStelle != 0) {
            istIndiziert = true;
            aktiverIndex = letzteStelle;
        }
    }

    // --- SCHICHT 2: DER TÜRSTEHER & BEREINIGUNG (Wenn der Index-Modus aktiv ist) ---
    if (istIndiziert) {
        uint16_t testId = originaleId - aktiverIndex;
        uint8_t testLetzteStelle = testId & 0x000F;
        
        // Gültig, wenn die bereinigte ID auf 0 endet ODER Kapazität (0x0211) ist
        bool istGueltig = (testLetzteStelle == 0) ; //|| ((testId & 0xFFF0) == 0x0210 && testLetzteStelle == 1);
        
        if (!istGueltig) {
            return 0; // ⛔ Ungültig! Signalisiert der Hauptschleife: Paket verwerfen!
        }
        
        return testId; // 🟢 Gültig! Gibt die geradegebogene ID zurück (z.B. 0x0110 statt 0x0112)
    }

    // Wenn das System im normalen Standard-Modus läuft, ID einfach so zurückgeben
    return originaleId;
}


// INDEX Überprüfen
uint16_t filterFrSkyId(uint16_t originaleId) {
/**
 * Verarbeitet die FrSky-ID und filtert/bereinigt sie basierend auf dem erkannten Index.
 * 
 * @param originaleId Die unveränderte ID aus dem Datenpaket
 * @return Die bereinigte ID für das Switch-Case, oder 0 wenn das Paket verworfen werden muss.
 */

    uint16_t basisId = originaleId & 0xFFF0;

    // GPS und andere IDs ab 0x0800 werden ungefiltert durchgewinkt
    // AUSNAHME: 0x0880, 0x0A10, 0x0B50, 0x0B60, 0x0B70, 0x0E50 sowie 0x0300 laufen durch den Index-Filter!
    if ((originaleId >= 0x0800 && 
         basisId != 0x0880 && 
         basisId != 0x0A10 && 
         basisId != 0x0B50 && 
         basisId != 0x0B60 && 
         basisId != 0x0B70 && 
         basisId != 0x0E50) || basisId == 0x0300) {
        return originaleId;
    }

    uint8_t letzteStelle = originaleId & 0x000F;

    // --- SCHICHT 1: INDEX-ERKENNUNG (Nur beim allerersten indizierten Paket) ---
    if (!istIndiziert) {
        // Standard-Gruppen: 0x0100 (Höhe), 0x0110 (Vario), 0x0200 (Strom), 0x0210 (MVOLT)
        if (basisId == 0x0100 || basisId == 0x0110 || basisId == 0x0200 || basisId == 0x0210) {
            if (letzteStelle != 0) {
                istIndiziert = true;
                aktiverIndex = letzteStelle; // z.B. 0x0102 -> Index 2
            }
        }
        // Sicherheitsnetz für alle anderen durchgelassenen IDs (inkl. 0x0B50, 0x0B60, 0x0B70, 0x0E50)
        else if (letzteStelle != 0) {
            istIndiziert = true;
            aktiverIndex = letzteStelle;
        }
    }

    // --- SCHICHT 2: DER TÜRSTEHER & BEREINIGUNG (Wenn der Index-Modus aktiv ist) ---
    if (istIndiziert) {
        uint16_t testId = originaleId - aktiverIndex;
        uint8_t testLetzteStelle = testId & 0x000F;
        
        // Gültig, wenn die bereinigte ID auf 0 endet
        bool istGueltig = (testLetzteStelle == 0);
        
        if (!istGueltig) {
            return 0; // Ungültig! Signalisiert der Hauptschleife: Paket verwerfen!
        }
        
        return testId; // Gültig! Gibt die geradegebogene ID zurück (z.B. 0x0B50 statt 0x0B51)
    }

    // Wenn das System im normalen Standard-Modus läuft, ID einfach so zurückgeben
    return originaleId;
}




// GA CELLS
void decodeFrSkyCells(int32_t val) {
    extern volatile uint16_t rx_cells_mvolt[];
    extern bool has_rx_cells;
    extern int32_t live_mvolt;
    extern bool has_mvolt;

    uint8_t raw_byte0 = val & 0xFF;
    uint16_t cellA_volt = ((val >> 8) & 0x0FFF) * 2; 
    uint16_t cellB_volt = ((val >> 20) & 0x0FFF) * 2;
    uint8_t packet_id = raw_byte0 & 0x0F;

    if (packet_id == 0) {
        rx_cells_mvolt[0] = cellA_volt; // Zelle 1
        rx_cells_mvolt[1] = cellB_volt; // Zelle 2
        
        // 🟢 BRÜCKE ZU MSTRENS' SYSTEM: oXs-RAM-Töpfe füllen und Flags aktivieren!
        setField(fields[ADS_1_1], cellA_volt); 
        setField(fields[ADS_1_2], cellB_volt); 
    } 
    else if (packet_id == 2) {
        rx_cells_mvolt[2] = cellA_volt; // Zelle 3
        rx_cells_mvolt[3] = cellB_volt; // Zelle 4
        
        // 🟢 BRÜCKE ZU MSTRENS' SYSTEM: oXs-RAM-Töpfe füllen und Flags aktivieren!
        setField(fields[ADS_1_3], cellA_volt);
        setField(fields[ADS_1_4], cellB_volt);
    } 
    else if (packet_id == 4) {
        rx_cells_mvolt[4] = cellA_volt; // Zelle 5
        rx_cells_mvolt[5] = cellB_volt; // Zelle 6
        
        // 🟢 BRÜCKE ZU MSTRENS' SYSTEM: oXs-RAM-Töpfe füllen und Flags aktivieren!
        setField(fields[ADS_2_1], cellA_volt);
        setField(fields[ADS_2_2], cellB_volt);
    }
    else if (packet_id == 6) {
        rx_cells_mvolt[6] = cellA_volt; // Zelle 7
        rx_cells_mvolt[7] = cellB_volt; // Zelle 8
        
        // 🟢 BRÜCKE ZU MSTRENS' SYSTEM: Auf 8 Zellen erweitert über ADS_2_3 und ADS_2_4!
        setField(fields[ADS_2_3], cellA_volt);
        setField(fields[ADS_2_4], cellB_volt);
    }
    
    has_rx_cells = true;

    // Gesamtspannung berechnen
    int32_t total_mvolt = 0;
    for (int i = 0; i < 8; i++) {
        if (rx_cells_mvolt[i] > 2000) { 
            total_mvolt += rx_cells_mvolt[i];
        }
    }

    //if (total_mvolt > 0) {
        // live_mvolt = total_mvolt;
        // has_mvolt = true;
       
    if ( ! has_mvolt)  setField(fields[MVOLT], total_mvolt);
    setField(fields[RESERVE1], total_mvolt);
    
    
   // }
}

// VARIO ALTITUDE
void decode0x100(int32_t val) {// 🏔️ FrSky Vario: Absolute Höhe -> in relative Höhe umrechnen
extern int32_t live_alt_rel;
static int32_t alt_offset = 0;
static uint32_t telemetry_start_time = 0;
static bool first_telemetry_received = false;
static bool offset_captured = false;
extern bool has_alt,has_alt_rel,has_alt_uni;

/*
if (has_alt_uni ) {  return; }// Wenn UNI-Höhe aktiv ist, IGNORIERE das FrSky Vario-Paket komplett!


// 🟢 Schritt 1: Wir warten, bis ÜBERHAUPT das erste Mal ein echter Wert reinkommt
if (!first_telemetry_received) {
    if (val != 0 && val != -1) { // -1 und 0 filtern (Protokoll-Rauschen beim Einschalten)
        telemetry_start_time = to_ms_since_boot(get_absolute_time()); // Timer JETZT ERST starten!
        first_telemetry_received = true;
    }
    // Solange kein echtes Signal da ist, geben wir zur Sicherheit 1000 aus
    setField(fields[RELATIVEALT], 1000);
    setField(fields[ALTITUDE], 1000);  
    return; // Funktion für dieses Paket abbrechen, bis echte Daten da sind
}

uint32_t time_elapsed = to_ms_since_boot(get_absolute_time()) - telemetry_start_time;

// 🟢 Phase 1: Die 10 Sekunden Aufwärmphase LAUFEN AB JETZT (Signal ist da!)
if (time_elapsed < 5000) {
    live_alt_rel = 1000; 
    setField(fields[RELATIVEALT], 1000); // Zeigt stabil 10m im Display während des Einmessens
    setField(fields[ALTITUDE], 1000);  
} 
// 🟢 Phase 2: Die 5 Sekunden echten Datenflusses sind um -> Offset fixieren
else {
    // Genau einmalig nach den 10 Sekunden den nun absolut stabilen Warm-Wert greifen
    if (!offset_captured) {
        if (val != 0) {
            alt_offset = val;
            offset_captured = true;
        }
    }

    // Ab jetzt die echte relative Höhe berechnen (Startet perfekt bei 0)
    live_alt_rel = val - alt_offset; 
    setField(fields[RELATIVEALT], live_alt_rel); 
    setField(fields[ALTITUDE], live_alt_rel);  
}
*/
has_alt_rel = true;
    setField(fields[RELATIVEALT], val) ; //); // BARO
   // setField(fields[ALTITUDE], val * 100);  

}
// Strom Kapazität
void decode0x200(int32_t val) {
    // 1. Zeitstempel über die in deinem Projekt genutzte Funktion holen
    static uint32_t last_time_ms = 0;
    uint32_t current_time_ms = millisRp(); 
    
    setField(fields[CURRENT], val);

    // Statischer Float-Speicher für präzise mAh-Schritte
    static float accumulated_mah = 0.0f;

    // 2. Zeitdifferenz berechnen (nur wenn es nicht der erste Aufruf ist)
    if (last_time_ms > 0 && current_time_ms > last_time_ms) {
        uint32_t delta_t_ms = current_time_ms - last_time_ms;
        
        // Formel: (Strom in mA * Zeit in ms) / 3.600.000
        accumulated_mah += ((float)val * (float)delta_t_ms) / 3600000.0f;
    }
    
    // Zeitstempel für den nächsten Durchlauf merken
    last_time_ms = current_time_ms;

    // 3. Float in Ganzzahl konvertieren und an oXs übergeben
    int32_t val2 = (int32_t)accumulated_mah;

    // Übergabe an oXs (Index CAPACITY)
    setField(fields[CAPACITY], val2);
}



// =========================================================================
// MODUS 1: DIREKT-RTH (Fliegt auf Home zu, innerhalb min_dist auf Neutral 5000)
// =========================================================================
int32_t calcDirectRTH(float distance_m, float min_distance_m, float bearing_from_home, float track_deg) {
    if (distance_m <= min_distance_m) {
        return 0; // Im Nahbereich sicher auf Neutral (Mitte)
    }

    float bearing_to_home = bearing_from_home + 180.0f;
    if (bearing_to_home >= 360.0f) bearing_to_home -= 360.0f;

    float turn_angle = bearing_to_home - track_deg;
    if (turn_angle > 180.0f)  turn_angle -= 360.0f;
    if (turn_angle < -180.0f) turn_angle += 360.0f;

    //return (int32_t)(((turn_angle / 3.6f) + 50.0f) * 100.0f); // Reduzierung auf 0 bis 100
    return (int32_t)(turn_angle  * 100.0f);
}

// =========================================================================
// MODUS 2: KREISBAHN / ORBIT (Geometrische Tangente im Uhrzeigersinn, 1 Hz)
// =========================================================================
int32_t calcOrbitRTH(float distance_m, float target_radius_m, float bearing_from_home, float track_deg, float min_speed_kmh) {
    // 1. Dynamischer Pilotenschutz: Passt sich automatisch an RTS an
    // Auto (RTS <= 2) -> 1.5m bis 2.4m | Segler (RTS = 10) -> 12.0m Schutz
    float pilot_protection_m = min_speed_kmh * 1.2f;
    if (pilot_protection_m < 1.5f) pilot_protection_m = 1.5f;

    if (distance_m < pilot_protection_m) {
        return 0;
    }

    // 2. Direkte Richtung vom Modell ZUM Startpunkt (Home) berechnen (0..360°)
    float bearing_to_home = bearing_from_home + 180.0f;
    if (bearing_to_home >= 360.0f) bearing_to_home -= 360.0f;

    // 3. Tangenten-Winkel (alpha) zum Kreisrand bestimmen
    float tangent_offset_deg = 0.0f;

    if (distance_m > target_radius_m) {
        // AUSSERHALB DES KREISES: Direkte Tangente zum Kreisrand mit arcsin(R / D)
        float ratio = target_radius_m / distance_m;
        if (ratio > 1.0f) ratio = 1.0f;
        tangent_offset_deg = asinf(ratio) * 57.29578f;
        
        // Begrenzung auf 80° (10° Vorhalt nach innen kompensiert Fliehkraft & 1-Hz-Takt)
        if (tangent_offset_deg > 80.0f) tangent_offset_deg = 80.0f;
    } else {
        // INNERHALB DES KREISES: Sanft nach außen auf den Kreisrand drücken (80° bis 135°)
        float inside_ratio = (target_radius_m - distance_m) / target_radius_m;
        tangent_offset_deg = 20.0f + (inside_ratio * 45.0f);
    }

    // 4. Zielkurs festlegen (Minus für Rechtskreis im Uhrzeigersinn)
    float target_heading = bearing_to_home - tangent_offset_deg;
    if (target_heading < 0.0f)    target_heading += 360.0f;
    if (target_heading >= 360.0f) target_heading -= 360.0f;

    // 5. Reinen Steuerwinkel ermitteln (-180° bis +180°) und auf 0..10000 skalieren
    float turn_angle = target_heading - track_deg;
    if (turn_angle > 180.0f)  turn_angle -= 360.0f;
    if (turn_angle < -180.0f) turn_angle += 360.0f;

    //return (int32_t)(((turn_angle / 3.6f) + 50.0f) * 100.0f); Reduzierung auf 0 bis 100
    return (int32_t)(turn_angle  * 100.0f);
}


//GPS Position & RTH
void decode0x800(int32_t val,  uint8_t buffer[10] ) { // GPS Koordinaten

    // 🟢 DOMINANTE VARIABLE: Hier kannst du den Sicherheitsradius (in Metern) jederzeit blitzschnell ändern!
    static float min_distance_m = config.rth_min_Dist;// 60.0f; // Für große Segler empfohlen: 30.0f bis 50.0f Meter
    static float min_speed = config.rth_min_Speed;// 2.0f; // Minimale geschwindigkeit sollte auf 4 sein
    static int32_t current_heading_cmd = 5000; // Startwert: Exakt 50.00 (Neutral)

    extern int32_t live_alt, live_mvolt, live_current, live_speed, live_sat, live_cap, live_pitch, live_roll, live_heading, live_rpm, live_lat, live_lon, live_alt_rel;
    extern bool has_alt, has_alt_rel, has_mvolt, has_current, has_speed, has_sat, has_rpm, has_cap, has_lat, has_lon, has_pitch, has_roll, has_head;

    // 🟢 FIX: Indizes, [6], [7], [8] wieder korrekt hinzugefügt!
    uint32_t raw_bytes = ((uint32_t)buffer[5])        | 
                         ((uint32_t)buffer[6] << 8)   | 
                         ((uint32_t)buffer[7] << 16)  | 
                         ((uint32_t)buffer[8] << 24);

    // Hilfs-Flags, um zu erkennen, wenn ein neues Paar da ist
    static bool new_lat_received = false;
    static bool new_lon_received = false;

    // Bit 31 prüfen: 0 = Latitude, 1 = Longitude
    bool is_longitude = (raw_bytes & 0x80000000UL) != 0;
    // Bit 30 prüfen: 0 = Nord/Ost (+), 1 = Süd/West (-)
    bool is_negative  = (raw_bytes & 0x40000000UL) != 0;
    // Die unteren 30 Bit sind der reine Koordinatenwert
    uint64_t pure_coord = (uint64_t)(raw_bytes & 0x3FFFFFFFUL);

    int32_t next_coord = (int32_t)((pure_coord * 50ULL) / 3ULL);
    if (is_negative) {
        next_coord = -next_coord;
    }

    if (!is_longitude) { 
        // Breitengrad (Latitude)
        if (fields[LATITUDE].value == 0 || abs(next_coord - fields[LATITUDE].value) < 100000) {
            setField(fields[LATITUDE], next_coord);          
            has_lat = true;
            new_lat_received = true; 
        }
    } 
    else { 
        // Längengrad (Longitude)
        if (fields[LONGITUDE].value == 0 || abs(next_coord - fields[LONGITUDE].value) < 100000) {
            setField(fields[LONGITUDE], next_coord); 
            has_lon = true;
            new_lon_received = true; 
        }
    }

    // Definitionen für Home-Location und Qualitätschallenger
    static int32_t home_lat = 0;
    static int32_t home_lon = 0;
    static uint64_t stable_start_us = 0; 
    static int32_t min_lat = 2147483647, max_lat = -2147483648;
    static int32_t min_lon = 2147483647, max_lon = -2147483648;
    static bool home_is_locked = false; 

    uint64_t current_us = to_us_since_boot(get_absolute_time()); 

    // Weltweiter Plausibilitätscheck
    bool gps_is_valid = false;
    if (fields[LATITUDE].value != 0 && fields[LONGITUDE].value != 0 &&
        fields[LATITUDE].value > -900000000 && fields[LATITUDE].value < 900000000 &&
        fields[LONGITUDE].value > -1800000000 && fields[LONGITUDE].value < 1800000000) {
        gps_is_valid = true; 
    }

    // REINE MATHEMATISCHE QUALITÄTS-PRÜFUNG AM BODEN
    if (!home_is_locked && gps_is_valid) {
        if (stable_start_us == 0) {
            stable_start_us = current_us;
            min_lat = max_lat = fields[LATITUDE].value;
            min_lon = max_lon = fields[LONGITUDE].value;
        }

        if (fields[LATITUDE].value < min_lat)  min_lat = fields[LATITUDE].value;
        if (fields[LATITUDE].value > max_lat)  max_lat = fields[LATITUDE].value;
        if (fields[LONGITUDE].value < min_lon) min_lon = fields[LONGITUDE].value;
        if (fields[LONGITUDE].value > max_lon) max_lon = fields[LONGITUDE].value;

        int32_t current_drift_lat = max_lat - min_lat;
        int32_t current_drift_lon = max_lon - min_lon;

        if (current_drift_lat <= 300 && current_drift_lon <= 300) {
            if (current_us - stable_start_us >= 9000000ULL) { 
                home_lat = min_lat + (current_drift_lat / 2);
                home_lon = min_lon + (current_drift_lon / 2);

                setField(fields[NUMSAT], 106);

                live_sat = 106; 
                home_is_locked = true; 
                has_sat = true;
            } else {
                setField(fields[NUMSAT], 56);
            }
        } 
        else {
            stable_start_us = current_us;
            min_lat = max_lat = fields[LATITUDE].value;
            min_lon = max_lon = fields[LONGITUDE].value;
        }


    }

    // Solange Home noch nicht gelockt ist, aktiv den Startwert 5000 senden
    if (!home_is_locked) {
           current_heading_cmd = 0;
          // setField(fields[HEADING], current_heading_cmd);
           setField(fields[GPS_HOME_BEARING], 0);
    }

    // =========================================================================
    // BERECHNUNG NUR WENN EIN VOLLES NEUES KOORDINATENPAAR DA IST
    // =========================================================================
    if (new_lat_received && new_lon_received) {
        
        // Flags sofort zurücksetzen für das nächste Paar
        new_lat_received = false;
        new_lon_received = false;

        // Statische Variablen für das GPS-Delta (Flugrichtungs-Ermittlung)
        static int32_t last_lat = 0;
        static int32_t last_lon = 0;

        float distance_m = 0.0f;

        // 1. Entfernung berechnen (Home Distance)
        if (home_lat != 0 && home_lon != 0) {
            float dist_lat = (float)(fields[LATITUDE].value - home_lat) * 0.0111139f;
            float dist_lon = (float)(fields[LONGITUDE].value - home_lon) * 0.0111139f * 0.6067f; 

            distance_m = sqrtf((dist_lat * dist_lat) + (dist_lon * dist_lon));
            setField(fields[GPS_HOME_DISTANCE], (int32_t)distance_m);
            setField(fields[TEMP1], (int32_t)distance_m);
     
            // 2. Home-Bearing berechnen und in HEADING schreiben
            float home_bearing = 90.0f + atan2f(-dist_lat, dist_lon) * 57.2957795f;
            if (home_bearing < 0.0f) {
                home_bearing += 360.0f;
            }
            setField(fields[HEADING], (int32_t)home_bearing);
            fields[NUMSAT].value = 12;
            live_sat = 12;
            has_sat = true;
        }


        // 2. Berechnung des Steuerkurses nach Hause
         if (home_is_locked) {
            
            // Aktuelle Config-Werte holen (mit sicheren Fallbacks, falls 0)
            float active_min_dist  = (config.rth_min_Dist > 0)  ? config.rth_min_Dist  : 60.0f;
            float active_min_speed = (config.rth_min_Speed > 0) ? config.rth_min_Speed : 4.0f;


            bool  use_orbit_mode   = config.rth_orbit; //true; // Oder über config steuern, z.B. (config.rth_mode == 1)

            // Automatische Breitengrad-Skalierung (funktioniert weltweit, nicht nur bei 0.6067)
            float lat_scale = cosf((float)home_lat * 1.0e-7f * 0.01745329f);

            float current_speed_kmh = ((float)fields[GROUNDSPEED].value * 36.0f) / 1000.0f;

            if (last_lat == 0 || last_lon == 0) {
                last_lat = fields[LATITUDE].value;
                last_lon = fields[LONGITUDE].value;
            }

            float move_lat = (float)(fields[LATITUDE].value - last_lat) * 0.0111139f;
            float move_lon = (float)(fields[LONGITUDE].value - last_lon) * 0.0111139f * lat_scale;
            float step_dist_m = sqrtf((move_lat * move_lat) + (move_lon * move_lon));

            // Sonderfall Direkt-RTH im Nahbereich sofort auf 5000 setzen (auch ohne Bewegung)
            if (!use_orbit_mode && distance_m <= active_min_dist) {
                current_heading_cmd = 0;
               // setField(fields[HEADING], current_heading_cmd);
                setField(fields[GPS_HOME_BEARING], 0);

                last_lat = fields[LATITUDE].value;
                last_lon = fields[LONGITUDE].value;
            }
                        // Reguläre Berechnung bei Fahrt (> RTS) und >= 3.5m Schrittweite
            else if (current_speed_kmh > active_min_speed && step_dist_m >= 3.5f) {
                static float smooth_move_lat = 0.0f;
                static float smooth_move_lon = 0.0f;

                // Glättet die FrSky-Treppenstufen (60% neuer Schritt, 40% vorheriger Schritt)
                smooth_move_lat = (smooth_move_lat * 0.4f) + (move_lat * 0.6f);
                smooth_move_lon = (smooth_move_lon * 0.4f) + (move_lon * 0.6f);

                float dLat_calc = (float)(fields[LATITUDE].value - home_lat);
                float dLon_calc = (float)(fields[LONGITUDE].value - home_lon) * lat_scale;
                float bearing_from_home = atan2f(dLon_calc, dLat_calc) * 57.29578f;
                if (bearing_from_home < 0.0f) bearing_from_home += 360.0f;

                float track_deg = atan2f(smooth_move_lon, smooth_move_lat) * 57.29578f;
                if (track_deg < 0.0f) track_deg += 360.0f;

                int32_t target_cmd = 0;
                if (use_orbit_mode) {
                    target_cmd = calcOrbitRTH(distance_m, active_min_dist, bearing_from_home, track_deg, active_min_speed);
                } else {
                    target_cmd = calcDirectRTH(distance_m, active_min_dist, bearing_from_home, track_deg);
                }

                current_heading_cmd = target_cmd / 18;
                setField(fields[GPS_HOME_BEARING], current_heading_cmd);

                last_lat = fields[LATITUDE].value;
                last_lon = fields[LONGITUDE].value;
            }

            // Reguläre Berechnung bei Fahrt (> min_speed) und >= 3.5m Schrittweite alt
            else if (1>1) {
                
                // 1. Peilung von Home zum Modell (0..360°)
                float dLat_calc = (float)(fields[LATITUDE].value - home_lat);
                float dLon_calc = (float)(fields[LONGITUDE].value - home_lon) * lat_scale;
                float bearing_from_home = atan2f(dLon_calc, dLat_calc) * 57.29578f;
                if (bearing_from_home < 0.0f) bearing_from_home += 360.0f;

                // 2. Aktueller Flugkurs (0..360°)
                float track_deg = atan2f(move_lon, move_lat) * 57.29578f;
                if (track_deg < 0.0f) track_deg += 360.0f;

                // 3. Gewählten Modus aufrufen
                int32_t target_cmd = 0;
                if (use_orbit_mode) {
                    target_cmd = calcOrbitRTH(distance_m, active_min_dist, bearing_from_home, track_deg, active_min_speed);
                } else {
                    target_cmd = calcDirectRTH(distance_m, active_min_dist, bearing_from_home, track_deg);
                }

                // 4. Schneller 1-Hz-Filter (70% neu, 30% alt)
                // current_heading_cmd = (current_heading_cmd * 3 + target_cmd * 7) / 10;
                // setField(fields[HEADING], current_heading_cmd);
                current_heading_cmd = (target_cmd) / 18;
                setField(fields[GPS_HOME_BEARING], current_heading_cmd);

                last_lat = fields[LATITUDE].value;
                last_lon = fields[LONGITUDE].value;
            }
            // Geschwindigkeit unter Mindestwert (RTS) -> Ruder sicher auf Neutral (0)
            else  {  // if (current_speed_kmh <= active_min_speed) 
                current_heading_cmd = 0;
                setField(fields[GPS_HOME_BEARING], 0);
            }
        }




    } else {
       // setField(fields[HEADING], 5000);

    }

}

// GPS ALTITUDE
void decode0x820(int32_t val) { // GPS Höhe

    extern bool has_alt, has_alt_rel, GpsAlt, has_sat, has_lon, has_alt_uni;
    extern int32_t live_alt;
    static int32_t alt_offset = 0;
    static uint32_t telemetry_start_time = 0;
    static bool first_telemetry_received = false;
    static bool offset_captured = false;
    static bool sat_captured = false;
    // 🟢 Schritt 1: Wir warten, bis ÜBERHAUPT das erste Mal ein echter Wert reinkommt
    GpsAlt = true;
    has_alt = true;

    //setField(fields[GROUNDSPEED], val * 100);

    setField(fields[ALTITUDE], val );  // GPS
 
    return;  

    if  ( has_alt_rel  || has_alt_uni ){  return; }
    
    //setField(fields[ALTITUDE], fields[RELATIVEALT].value); 
      
    return;
   

    if (!first_telemetry_received) {
        if (val != 0 && val != -1) { // -1 und 0 filtern (Protokoll-Rauschen beim Einschalten)
          telemetry_start_time = to_ms_since_boot(get_absolute_time()); // Timer JETZT ERST starten!
          first_telemetry_received = true;
        }
        // Solange kein echtes Signal da ist, geben wir zur Sicherheit 1000 aus
        live_alt = 1000;
   
    } else {

        uint32_t time_elapsed = to_ms_since_boot(get_absolute_time()) - telemetry_start_time;

        if (time_elapsed < 5000) {
          live_alt = 1000; 

        } else {
            // Genau einmalig nach den 10 Sekunden den nun absolut stabilen Warm-Wert greifen
            if (!offset_captured) {
               if (val != 0) {
                  alt_offset = val;
                  offset_captured = true;
                }
            }

            if (!sat_captured && has_sat) {
               if (val != 0) {
                  alt_offset = val;
                  sat_captured = true;
                }
            }

            // Ab jetzt die echte relative Höhe berechnen (Startet perfekt bei 0)
            live_alt = val - alt_offset; 
        }
    }
    
    setField(fields[ALTITUDE], live_alt);
    setField(fields[RELATIVEALT], live_alt); 
    //setField(fields[ALTITUDE], 2000); //live_alt);

}
// GPS SPEED
void decode0x830(int32_t val) {

    extern int32_t live_speed;
    int32_t speed_raw = val; // Echter Sensor-Rohwert aus dem Datenstrom
    // 1 Knoten = 51.44 cm/s. Da raw=Knoten*100 gilt: (speed_raw * 51) / 1000
    int32_t speed_cm_s = (speed_raw * 514) / 10000;
    // 30 cm/s entspricht ca. 1 km/h. Alles darunter wird im Stand genullt.
    if (speed_cm_s < 30) {
        speed_cm_s = 0;
    }

    // Gleitender Tiefpassfilter (Glättet die Anzeige für stabile Werte)
    static int32_t filtered_speed = 0;
    filtered_speed = ((filtered_speed * 3) + speed_cm_s) / 4;

    // Übergabe der glatten, korrekten cm/s an die HoTT-Variablen & oXs
    live_speed = filtered_speed; 
    //has_speed = true;
                                    
    setField(fields[GROUNDSPEED], speed_cm_s); //has_speed = true ; 

}
// GPS HEADING
void decode0x840(int32_t val) {                       // GPS: Kurs (Heading)
   extern int32_t live_sat;
   if (live_sat >= 4) { fields[HEADING].value = val; }// GPS: Kurs (Heading)
    
}
// GPS Zeit
void decode0x850(int32_t val,  uint8_t buffer[10] ) {
               uint8_t subID = buffer[5]; // B5 bestimmt den Modus (00 = Zeit, 01 = Datum)

                if (subID == 0x00) {
                    // --- UHRZEIT EXTRAHIEREN ---
                    uint8_t second = buffer[6]; // B6 sind die Sekunden
                    uint8_t minute = buffer[7]; // B7 sind die Minuten
                    uint8_t hour   = 0;         // Stunde ist bei iNav am Tisch oft 0

                    // 🟢 EUROPA/BERLIN-RECHNUNG DIREKT BEIM EINLESEN
                    // Wir addieren +2 Stunden für Mitteleuropäische Sommerzeit (CEST)
                    // (Wenn Winterzeit herrscht, hier einfach eine +1 eintragen)
                    int32_t local_hour = hour + 1; 
                    if (local_hour >= 24) {
                        local_hour -= 24;
                    }

                    // oXs-Format bauen (HHMMSS00) mit der neuen Lokalzeit
                    uint32_t oXs_time = ((uint32_t)local_hour * 1000000) + 
                                        ((uint32_t)minute * 10000) + 
                                        ((uint32_t)second * 100);
                    
                    setField(fields[GPS_TIME], oXs_time);                    
                } 
                else if (subID == 0x01) {
                    // --- DATUM EXTRAHIEREN ---
                    uint8_t day   = buffer[6]; // B6 ist der Tag (14)
                    uint8_t month = buffer[7]; // B7 ist der Monat (10)
                    uint8_t year  = 26;        // Das Jahr frieren wir sicherheitshalber auf 26 ein

                    // oXs-Format bauen (DDMMYY00)
                    uint32_t oXs_date = ((uint32_t)day * 1000000) + 
                                        ((uint32_t)month * 10000) + 
                                        ((uint32_t)year * 100);

                    setField(fields[GPS_DATE], oXs_date);
                }


 }// GPS Zeit
// UNI ALTITUDE
void decode0x880(int32_t val) {// 🏔️ FrSky Vario: Absolute Höhe -> in relative Höhe umrechnen

    extern bool has_alt,has_alt_rel,has_alt_uni;

     uint32_t ALT = val;
    

    //setField(fields[RELATIVEALT], 333); 
     setField(fields[RELATIVEALT], ALT );   // BARO
     setField(fields[ALTITUDE], ALT );      // GPS

    has_alt_rel = true;
    has_alt_uni = true;
}

void decode0xB50(int32_t val) { // FrSky ESC: Akkuspannung & Motorstrom
    uint32_t rawVolt = (uint32_t)(val & 0xFFFF);         // Untere 16 Bit (in 0.01 V)
    uint32_t rawCurr = (uint32_t)((val >> 16) & 0xFFFF); // Obere 16 Bit (in 0.01 A)

    int32_t mVolt = rawVolt * 10; // Umrechnung 0.01 V -> mV (für oXs MVOLT)
    int32_t mAmp  = rawCurr * 10; // Umrechnung 0.01 A -> mA (für oXs CURRENT)

    setField(fields[MVOLT], mVolt);
    setField(fields[CURRENT], mAmp);
}

void decode0xB60(int32_t val) { // FrSky ESC: Motordrehzahl & Verbrauch (mAh)
    uint32_t rawRpm  = (uint32_t)(val & 0xFFFF);         // Untere 16 Bit (in 100 U/min)
    uint32_t rawCons = (uint32_t)((val >> 16) & 0xFFFF); // Obere 16 Bit (in mAh)

    int32_t rpm      = rawRpm * 100; // Umrechnung 100 U/min -> U/min (für oXs RPM)
    int32_t capacity = rawCons;      // Bereits in mAh (für oXs CAPACITY)

    setField(fields[RPM], rpm);
    setField(fields[CAPACITY], capacity);
}

void decode0xB70(int32_t val) { // FrSky ESC: Reglertemperatur
    int16_t tempC = (int16_t)(val & 0xFFFF); // Untere 16 Bit vorzeichenbehaftet (in °C)

    setField(fields[TEMP1], (int32_t)tempC);
}

void decode0xE50(int32_t val) { // FrSky SBEC: BEC-Spannung & BEC-Strom
    uint32_t rawBecVolt = (uint32_t)(val & 0xFFFF);         // Untere 16 Bit (in 0.01 V)
    uint32_t rawBecCurr = (uint32_t)((val >> 16) & 0xFFFF); // Obere 16 Bit (in 0.01 A)

    int32_t becMVolt = rawBecVolt * 10; // Umrechnung 0.01 V -> mV (z.B. auf RESERVE1 / V3)
    int32_t becMAmp  = rawBecCurr * 10; // Umrechnung 0.01 A -> mA (z.B. auf RESERVE2 / V4)

    setField(fields[RESERVE1], becMVolt);
    setField(fields[RESERVE2], becMAmp);
}


void handleMultiProtocol_N(uint8_t b) {
    // 1. Alle externen globalen Variablen sauber anmelden
    extern int32_t live_alt, live_mvolt, live_current, live_speed, live_sat, live_cap, live_pitch, live_roll, live_heading, live_rpm, live_lat, live_lon, live_alt_rel;
    extern uint8_t live_flight_mode, live_is_armed;
    extern bool has_alt, has_alt_rel, has_mvolt, has_current, has_speed, has_sat, has_rpm, has_cap, has_lat, has_lon, has_pitch, has_roll, has_head;

    static enum { HUNT, HUNT_PHYSICAL, HUNT_PREFIX, DATA } state = HUNT;               
    static uint8_t buffer[10];                                          
    static uint8_t idx = 0;                                             
    static bool escape = false;                                         

    switch (state) {
        case HUNT:
            escape = false;                                             
            if (b == 0x7E) { buffer[0] = b; state = HUNT_PHYSICAL; }
#ifdef DEBUGINAV
            if (b == 0x7E) printf("[TRACE] -> HUNT_PHYSICAL (0x7E gefunden)\n");
#endif
            break;

        case HUNT_PHYSICAL:
            if (b == 0x7E) { buffer[0] = b; state = HUNT_PHYSICAL; break; }
            if (b == 0x7D) { state = HUNT; break; }
            buffer[1] = b; state = HUNT_PREFIX;                                        
            break;

        case HUNT_PREFIX:
            if (b == 0x7E) { buffer[0] = b; state = HUNT_PHYSICAL; break; }
            if (b == 0x7D) { state = HUNT; break; }                     
            buffer[2] = b; idx = 3; state = DATA;                                           
            break;

        case DATA:
            if (b == 0x7E) { buffer[0] = b; state = HUNT_PHYSICAL; break; }
            if (b == 0x7D) { escape = true; return; }                   
            if (escape) { b ^= 0x20; escape = false; }                  

            if (idx < 10) buffer[idx++] = b;
            
            if (idx == 10) {                                            
                uint32_t crc_check = 0;
                for (int i = 2; i < 9; i++) crc_check += buffer[i];
                while (crc_check > 0xFF) crc_check = (crc_check & 0xFF) + (crc_check >> 8);
                
                uint8_t calculated_crc = 0xFF - (uint8_t)crc_check;
                uint8_t received_crc = buffer[9];

                if (calculated_crc == received_crc) {
                    uint16_t id = (uint16_t)(buffer[3] | (buffer[4] << 8));             
                    int32_t val = (int32_t)((uint32_t)buffer[5] | ((uint32_t)buffer[6] << 8) | ((uint32_t)buffer[7] << 16) | ((uint32_t)buffer[8] << 24));            

                    if (id != 0 && id != 0xFFFF && id != 0x504D) {          
                        //  SCHRITT A: Dekodierung.

                        // neu indeciert
                        uint16_t verarbeitungsId = filterFrSkyId(id);

                        // Wenn die Funktion 0 zurückgibt, wurde das Paket blockiert -> Schleifendurchlauf abbrechen
                        if (verarbeitungsId == 0xFFFF) {
                             return; // Oder 'break;', je nachdem in was für einer Schleife du dich befindest
                        }

                         // Dein Switch-Case nutzt jetzt die sichere 'verarbeitungsId'
                        switch (verarbeitungsId) {
                        // neu indeciert

                        //switch (id) {

                            case 0x0100:              decode0x100(val)                                           ; break; // Höhe
                            case 0x0110:              setField(fields[VSPEED], val)                              ; break; // Vario
                            case 0x0200:              decode0x200(val * 100)                                     ; break; // FrSky 100mA -> HoTT 10mA
                           //case 0x0200:              setField(fields[CURRENT], val * 100); has_current = true  ; break; // FrSky 100mA -> HoTT 10mA
                            case 0x0210:              setField(fields[MVOLT], val * 10);    has_mvolt = true     ; break; // FrSky 100mV -> HoTT 10mV
                           // case 0x0600:              setField(fields[CAPACITY], val)                            ; break; // Kapazität
                            case 0x0300:              decodeFrSkyCells(val)                                      ; break; // Zellen
                            case 0x0500:              setField(fields[RPM], val);               has_rpm   = true ; break; // Drehzahl  
                            case 0x0511:              setField(fields[GROUNDSPEED], val * 100); has_speed = true ; break; // GPS Geschwindigkeit
 
                            case 0x0820:              decode0x820(val)                                           ; break; // GPS Höhe
                            case 0x0830:              decode0x830(val)                                           ; break; // GPS Speed
                            case 0x0840:              decode0x840(val)                                           ; break; // GPS Kurs (Heading)
                            case 0x0800:              decode0x800(val, buffer)                                   ; break; // GPS Koordinaten

                            case 0x0850:              decode0x850(val, buffer)                                   ; break; // GPS Zeit
                            //case 0x0880:              decode0x880(val)                                           ; break; // UNI Höhe

                            case 0x0B50:              decode0xB50(val)                                           ; break; // ESC Spannung & Strom (VFAS + Curr)
                            case 0x0B60:              decode0xB60(val)                                           ; break; // ESC Drehzahl & Verbrauch (RPM + mAh)
                            case 0x0B70:              decode0xB70(val)                                           ; break; // ESC Temperatur (Tmp1)
                            case 0x0E50:              decode0xE50(val)                                           ; break; // SBEC Spannung & Strom

 
                         }
                         //setField(fields[TEMP1], aktiverIndex );      // Unisens-E Index
                         setField(fields[TEMP2],  SW_Version  );        // SW-Version
                         fields[MVOLT].available = true;



                        // SCHRITT D: Debug!
#ifdef DEBUGINAV
                        dbg_hoehe = (float)live_alt / 100.0f;
                        dbg_vario = fields[VSPEED].value;
                        dbg_strom = (float)live_current / 100.0f;
                        dbg_volt  = (float)live_mvolt / 100.0f;
                        dbg_kap   = live_cap;
                        dbg_rpm   = live_rpm;

                        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
                        if (now_ms - dbg_last_print_ms >= 200) {
                            dbg_last_print_ms = now_ms;
                            printf("\r[DEBUGINAV] U: %.2fV | I: %.2fA | C: %ldmAh | H: %.1fm | V: %ldcm/s | R: %ldHz        ", 
                                   dbg_volt, dbg_strom, dbg_kap, dbg_hoehe, dbg_vario, dbg_rpm);
                            fflush(stdout);
                        }
#endif
                    }
                }
                state = HUNT;                                          
            }
           

            break;
    }
}



void init_frsky_sensors_core1(void) {
    
    FRSKY_PIN1 = config.inav_ltm_pin;

    gpio_init(FRSKY_PIN1);
    gpio_set_dir(FRSKY_PIN1, GPIO_OUT);
    gpio_put(FRSKY_PIN1, 0); 
    gpio_pull_down(FRSKY_PIN1); 
    gpio_set_outover(FRSKY_PIN1, GPIO_OVERRIDE_NORMAL);
    gpio_set_inover(FRSKY_PIN1, GPIO_OVERRIDE_NORMAL);
    active_sensors_count = 0;
    next_send_time_us = time_us_32();
}

void read_frsky_sensors_core1(void) {
    uint32_t now_us = time_us_32();
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    static uint8_t current_polling_id = 0; 

    static uint32_t last_packet_ms = 0;
    ltm_data_rx_flag = false;

    for (int i = 0; i < active_sensors_count; i++) {
        if (now_ms - last_seen_ms[i] > 2000) { unregister_sensor(i); break; }
    }

    if (!pin_is_listening && (now_us - next_send_time_us >= 12000)) {
        next_send_time_us = now_us; local_rx_idx = 0; 

        switch (scheduler_step) {
            case 0: 
                if (active_sensors_count > 0 && active_sensors[0] == 0x1B) {
                    current_polling_id = 0x1B; 
                } else {
                    current_polling_id = 0x00; 
                }
                break; 
            case 1: 
                if (active_sensors_count > 0) {
                    current_polling_id = active_sensors[class_b_rotation_idx];
                    class_b_rotation_idx++;
                    if (class_b_rotation_idx >= active_sensors_count) class_b_rotation_idx = 0;
                } else { 
                    current_polling_id = 0x1B; 
                }
                break; 
            case 2: 
                // 🚀 WECHSELTAKTIERUNG FÜR S.PORT GPS (ID 0x03)
                static bool toggle_gps_slot = false;
                
                if (toggle_gps_slot) {
                    current_polling_id = 0x03; // 🛰️ Festes Polling für das FrSky GPS!
                    toggle_gps_slot = false;
                } else {
                    // Originaler Klasse-A-Takt (Vario / iNav FC)
                    if (active_sensors_count > 0 && active_sensors[0] == 0x1B) {
                        current_polling_id = 0x1B; 
                    } else {
                        current_polling_id = 0x00; 
                    }
                    toggle_gps_slot = true;
                }
                break; 
            case 3: 
                current_polling_id = s_port_search_ids[search_idx];
                search_idx++;
                if (search_idx >= 28) search_idx = 0; 
                break;
        }

        gpio_set_dir(FRSKY_PIN1, GPIO_OUT);
        send_software_uart_byte(0x7E); 
        send_software_uart_byte(current_polling_id);
        
        sleep_us(140); 

        // =====================================================================
        // 🟢 DER ULTRA-DUNKEL-TRICK FÜR DEN LEERLAUF:
        // Wir lassen die Stufe 1 nur noch alle 120ms (Modulo 10) ganz kurz 
        // aufblitzen, den Rest der Zeit ist sie aus (0, 0, 0). 
        // Für das Auge ergibt das ein perfekt gedimmtes, mattes Glimmen!
        // =====================================================================
        if (now_ms - last_packet_ms >= 200) {
            if ((now_ms / 12) % 50 == 0) {
                // setRgbColorOn(0, 1, 0); // 🟢 Ganz kurz matt an
            } else {
                // setRgbColorOn(0, 0, 0); // Aus
            }
        } 
        
        //sleep_us(35); 
        gpio_set_dir(FRSKY_PIN1, GPIO_IN);
        gpio_pull_down(FRSKY_PIN1); 
        pin_listen_time_us = time_us_32(); pin_is_listening = true;
    }
    if (pin_is_listening) {
        uint8_t incoming_byte = 0;
        if (receive_software_uart_byte(&incoming_byte)) {
            if (local_rx_idx == 0) { local_rx_buf[local_rx_idx++] = 0x7E; }
            if (local_rx_idx < 25) { local_rx_buf[local_rx_idx++] = incoming_byte; }
        }

        if (time_us_32() - pin_listen_time_us >= 4000) {
            pin_is_listening = false; 

            if (local_rx_idx >= 9 && local_rx_buf[0] == 0x7E) {
                int reg_idx = get_sensor_registry_index(current_polling_id);
                if (reg_idx >= 0) { last_seen_ms[reg_idx] = now_ms; }
                else {
                    if (current_polling_id != 0x00 && active_sensors_count < MAX_ACTIVE_SENSORS) {
                        active_sensors[active_sensors_count] = current_polling_id;
                        last_seen_ms[active_sensors_count] = now_ms; active_sensors_count++;
                    }
                }

                // =============================================================
                // 🟢 HELLES BLITZEN IN GRÜN BEI ANTWORT:
                // Ein echtes Paket kam durch! Wir reißen die LED voll auf (Stufe 20).
                // =============================================================
                last_packet_ms = now_ms;
               // setRgbColorOn(0, 1, 0); // 🟢 Helles Grün für das Paket!

                if (debugFrS == 'Y') {
                    // 🟢 NATIVE LITTLE-ENDIAN RECHNUNG:
                    // ID-High (Index 3) bildet die linken 8 Bit, ID-Low (Index 2) bildet die rechten 8 Bit.
                    uint16_t real_id = (uint16_t)((local_rx_buf[3] << 8) | local_rx_buf[2]);

                    printf("\n[S.Port] (Anzahl=%d) | ID: 0x%04X : ", 
                           local_rx_idx, 
                           real_id);
                    
                    for (int i = 0; i < local_rx_idx; i++) { printf("%02X ", local_rx_buf[i]); }
                    printf("\n"); fflush(stdout);
                }           
#ifdef DEBUGINAV3
                printf("\n[S.Port] ");
                for (int i = 0; i < local_rx_idx; i++) { printf("%02X ", local_rx_buf[i]); }
                fflush(stdout);
#endif

                handleMultiProtocol_N(local_rx_buf[0]);
                handleMultiProtocol_N(local_rx_buf[1]);
                handleMultiProtocol_N(0x10);
                for (int i = 2; i < local_rx_idx; i++) { 
                    handleMultiProtocol_N(local_rx_buf[i]); 
                }

                // =============================================================
                // 🟢 DER RÜCKSPRUNG: Sobald die Daten verarbeitet sind, 
                // schalten wir SOFORT wieder aus. Das erzeugt das perfekte Blitzen!
                // =============================================================
                //setRgbColorOn(0, 0, 0); 

                //printAllTelemetryValues();
                ltm_data_rx_flag = true;
            } 
            else {
                // Wenn das Fenster abgelaufen ist und KEIN gültiges Paket kam,
                // schalten wir sie hier ebenfalls stur aus, damit sie glimmt.
                //setRgbColorOn(0, 0, 0); 
               ltm_data_rx_flag = false;
            }

            scheduler_step++;
            if (scheduler_step >= 4) scheduler_step = 0; 

        }
    }
}

typedef struct {
    int32_t current;   // Aktueller Simulationswert
    int32_t min;       // Minimaler Wert
    int32_t max;       // Maximaler Wert
    int32_t step;      // Schrittweite pro Durchlauf
    int8_t  direction; // Individuelle Richtung (+1 oder -1)
} SimConfig;

void simulate_frsky_sensors_core1()  {


    static uint32_t last_poll_time = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Aktualisierungsintervall (z.B. alle 50 ms)
    if (now - last_poll_time < 300) return; 
    last_poll_time = now;

    // 🟢 FLEXIBLE EINSTELLUNGEN PRO WERT [Startwert, Min, Max, Schrittweite, Richtung]
    static SimConfig cfg_alt         = {0,     -2000, 20000, 200, 1}; // ALTITUDE (in cm)
    static SimConfig cfg_vspeed      = {0,     -1000,  1000,  20, 1}; // VSPEED (in cm/s)
    static SimConfig cfg_current     = {15000,     0, 25000, 500, 1}; // CURRENT (in mA)
    static SimConfig cfg_capacity    = {0,         0,  5000,  10, 1}; // CAPACITY (in mAh)
    static SimConfig cfg_cell        = {4200,   3300,  4200,   5,-1}; // Einzelzellen (in mV)
    static SimConfig cfg_rpm         = {1200,   1000,  5000,  50, 1}; // RPM (in Hz/Umin)
    static SimConfig cfg_groundspeed = {0,         0,  3000,  40, 1}; // GROUNDSPEED (in cm/s)
    static SimConfig cfg_gps_alt     = {0,     -2000, 20000, 200, 1}; // GPS-Höhe (in cm)
    static SimConfig cfg_heading     = {18000,     0, 35900, 200, 1}; // HEADING (in 1/100 Grad)
    static SimConfig cfg_roll        = {0,     -1000,  1000, 100, 1}; // ROLL (in 1/100 Grad)
    static SimConfig cfg_pitch       = {-300,  -1000,  1000, 100, 1}; // PITCH (in 1/100 Grad)
    static SimConfig cfg_yaw         = {300 ,  -1000,  1000, 100, 1}; // YAW (in 1/100 Grad)

    // --- 1. SCHLEIFE: UPDATE DER WERTE INKL. GRENZPRÜFUNG ---
    #define UPDATE_SIM(cfg) do { \
        (cfg).current += ((cfg).step * (cfg).direction); \
        if ((cfg).current > (cfg).max || (cfg).current < (cfg).min) { \
            (cfg).direction = -(cfg).direction; \
            if ((cfg).current > (cfg).max) (cfg).current = (cfg).max; \
            if ((cfg).current < (cfg).min) (cfg).current = (cfg).min; \
        } \
    } while(0)

    UPDATE_SIM(cfg_alt);
    UPDATE_SIM(cfg_vspeed);
    UPDATE_SIM(cfg_current);
    UPDATE_SIM(cfg_capacity);
    UPDATE_SIM(cfg_cell);
    UPDATE_SIM(cfg_rpm);
    UPDATE_SIM(cfg_groundspeed);
    UPDATE_SIM(cfg_gps_alt);
    UPDATE_SIM(cfg_heading);
    UPDATE_SIM(cfg_roll);
    UPDATE_SIM(cfg_pitch);
    UPDATE_SIM(cfg_yaw);

    // --- 2. ÜBERGABE AN DAS VALIDIERTE OXS FIELDS-ARRAY ---

    // Höhenwerte (Vario)
    setField(fields[ALTITUDE], cfg_alt.current);
    setField(fields[RELATIVEALT], cfg_alt.current);

    setField(fields[VSPEED], cfg_vspeed.current);

    // Strom & Kapazität
    setField(fields[CURRENT], cfg_current.current);
    setField(fields[CAPACITY], cfg_capacity.current);

    
    // 🟢 Einzelzellen exakt berechnen (inklusive Drift)
    int32_t c1 = cfg_cell.current;
    int32_t c2 = cfg_cell.current - 150;
    int32_t c3 = cfg_cell.current + 500;
    int32_t c4 = cfg_cell.current - 100;
    int32_t c5 = cfg_cell.current;
    int32_t c6 = cfg_cell.current - 500;
    int32_t c7 = cfg_cell.current + 100;
    int32_t c8 = cfg_cell.current - 200;

    // Werte in die ADS-Felder schreiben
    rx_cells_mvolt[1] = c1;
    rx_cells_mvolt[2] = c2;
    rx_cells_mvolt[3] = c3;
    rx_cells_mvolt[4] = c4;
    rx_cells_mvolt[5] = c5;
    rx_cells_mvolt[6] = c6;
    setField(fields[ADS_1_1], c1);
    setField(fields[ADS_1_2], c2);
    setField(fields[ADS_1_3], c3);
    setField(fields[ADS_1_4], c4);
    setField(fields[ADS_2_1], c5);
    setField(fields[ADS_2_2], c6);
    //setField(fields[ADS_2_3], c7);
    //setField(fields[ADS_2_4], c8);

    // 🟢 Die echte mathematische Summe für MVOLT bilden
    int32_t total_mvolt = c1 + c2 + c3 + c4 + c5 + c6 ;//+ c7 + c8;

    has_rx_cells = true;

    setField(fields[MVOLT], total_mvolt);
    setField(fields[RESERVE1], total_mvolt);

    

    // Drehzahl
    //setField(fields[RPM], cfg_rpm.current);

    // GPS-Daten
    setField(fields[GROUNDSPEED], cfg_groundspeed.current);
    setField(fields[HEADING], cfg_heading.current);
    setField(fields[GPS_HOME_BEARING], cfg_heading.current);
    setField(fields[ROLL], cfg_roll.current);
    setField(fields[PITCH], cfg_pitch.current);
    setField(fields[YAW], cfg_yaw.current);
    setField(fields[GPS_HOME_DISTANCE], cfg_heading.current / 10);
    setField(fields[LATITUDE], 52638400);  // 52.638400° N
    setField(fields[LONGITUDE], 9983100);  // 9.983100° E
    
    // GPS-Höhe auf RESERVE2 abgelegt
   // setField(fields[RESERVE2], cfg_gps_alt.current);

    // Satellitenanzahl
    uint8_t sim_sats = (cfg_alt.direction > 0) ? 12 : 8;
    setField(fields[NUMSAT], sim_sats);
    //setField(fields[TEMP1], 2 );      // Unisens-E Index
   // setField(fields[TEMP2],  SW_Version * 10 + 2 );        // SW-Version

    // --- 3. HARDWARE-BLINKTAKTSYNC ---
    //if ((now / 1500) % 2 == 0) { setRgbColorOn(0, 1, 0); } else { setRgbColorOn(0, 0, 0); }
    // printAllTelemetryValues();*/
}


void test_gps_escaped_packet(void) {
    printf("\n--- STARTE S.PORT ESCAPE-PARSER TEST ---\n");

    // Der simulierte Bytestrom inklusive des gestopften 0x7D 0x5E Paars
    uint8_t test_stream[] = {
        0x7E,       // Start-Byte
        0x10,       // Frame Header (wird von deiner Lese-Funktion injiziert)
        0x03,       // Sensor-ID (GPS)
        0x00,       // Data ID Low (0x0800)
        0x08,       // Data ID High (0x0800)
        0x34,       // Value Byte 1 (Low)
        0x7D, 0x5E, // Das gestopfte Byte! Repräsentiert eigentlich 0x7E
        0x12,       // Value Byte 3
        0x00,       // Value Byte 4 (High) -> Gesamtwert ungestopft: 0x00127E34
        0x20        // CRC (berechnet auf Basis von 0xDF)
    };

    size_t stream_length = sizeof(test_stream) / sizeof(test_stream[0]);

    printf("Sende %d Bytes an handleMultiProtocol_N...\n", (int)stream_length);

    for (size_t i = 0; i < stream_length; i++) {
        handleMultiProtocol_N(test_stream[i]);
    }

    printf("--- TEST ABGESCHLOSSEN ---\n\n");
}