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
// 🟢 KORREKTUR: Wir nutzen den echten oXs-Typ FIELD (Großbuchstaben)
// Damit weiß der Compiler exakt, wie das oXs-Array aufgebaut ist.
extern field fields[]; 
int8_t FRSKY_PIN1 = 2; 
extern CONFIG config; 

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


void printAllTelemetryValues() {
     if (debugAll != 'Y') return;

    // Taktung fest auf 10-mal pro Sekunde (100 ms) einbremsen
    static uint32_t lastPrintTime = 0;
    uint32_t currentTime = to_ms_since_boot(get_absolute_time()); 

    if (currentTime - lastPrintTime < 300) { 
        return; 
    }
    lastPrintTime = currentTime; 

    // Prüfung auf die erste Zeile / den ersten Durchlauf
    static bool isFirstOutput = true;

    if (isFirstOutput) {
        // Beim allerersten Start machen wir KEINE Leerzeilen.
        // Die Tabelle beginnt direkt ganz oben im Terminal-Fenster.
        isFirstOutput = false; 
    } else {
        // Ab dem zweiten Durchlauf schieben wir den alten Block mit 
        // exakt 45 Leerzeilen nach oben aus dem Sichtfeld weg.
        for (uint8_t i = 0; i < 45; i++) {
            printf("\n");
        }
    }

    // Kopfzeile (beißt jetzt starr oben an)
    printf("=== oXs RP2040 Telemetry Live-Monitor (10 Hz) ===\n");
    printf("---------------------------------------------------\n");

    // Deine erweiterte Komma-Liste
    #define TELEMETRY_FIELDS \
        X(LATITUDE), X(LONGITUDE), X(ALTITUDE), X(RELATIVEALT), X(GROUNDSPEED), X(HEADING), \
        X(NUMSAT), X(GPS_DATE), X(GPS_TIME), X(GPS_HOME_DISTANCE), X(VSPEED), X(PITCH), X(ROLL), \
        X(YAW), X(RPM), X(CURRENT), X(CAPACITY), X(TEMP1), X(TEMP2), X(MVOLT), \
        X(ADS_1_1), X(ADS_1_2), X(ADS_1_3), X(ADS_1_4), X(ADS_2_1), X(ADS_2_2), X(ADS_2_3), X(ADS_2_4)

    // Das Auswertungs-Makro: Reiner Standard-Text untereinander.
    #define X(id) ([&]() { \
        if (fields[id].available) { \
            printf(" %-18s | Value: %-15ld\n", #id, (int32_t)fields[id].value); \
        } else if (fields[id].onceAvailable) { \
            printf(" %-18s | [Stale] %-15ld\n", #id, (int32_t)fields[id].value); \
        } else { \
            printf(" %-18s | [Not Available]\n", #id); \
        } \
        return 0; \
    }())

    // Array-Trick ausführen, damit der Compiler deine Liste zeilenweise abarbeitet
    int dummy[] = { TELEMETRY_FIELDS };
    (void)dummy; 

    printf("---------------------------------------------------\n");
    
    // Makros nach dem Durchlauf sofort wieder freigeben
    #undef X
    #undef TELEMETRY_FIELDS

    fflush(stdout); // Datenblock im Ganzen auf die USB-Schnittstelle schieben
}



// Definition (z. B. ganz oben in der Datei)
#define setField(f, v) do { \
    (f).value = (v); \
    (f).available = true; \
    (f).onceAvailable = true; \
    if (debugVal == 'Y') { \
        printf("\r %-25s | Value: %8ld        ", #f, (int32_t)(v)); \
        fflush(stdout); \
    } \
} while(0)


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

    if (total_mvolt > 0) {
        live_mvolt = total_mvolt;
        has_mvolt = true;
    }
}

void decode0x100(int32_t val) {// 🏔️ FrSky Vario: Absolute Höhe -> in relative Höhe umrechnen
    extern int32_t live_alt_rel;
    static int32_t alt_offset = 0;
    static bool alt_offset_stored = false;

    // Beim allerersten empfangenen Paket merken wir uns die Basishöhe
    if (!alt_offset_stored && val != 0) {
        alt_offset = val;
            alt_offset_stored = true;
        }

        // Ziehe die Basishöhe ab, um die echte relative Höhe (Start bei 0 cm) zu erhalten

    live_alt_rel = val - alt_offset; 
    //has_alt_rel = true;
    //setField(fields[CURRENT], 30000);
    setField(fields[RELATIVEALT], val - alt_offset); 

}


void decode0x800(int32_t val,  uint8_t buffer[10] ) { // GPS Koordinaten

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

                                    if (val > 0) { 
                                        // Positiver Wert -> Breitengrad (Latitude / Nord)
                                        uint64_t pure_lat = (uint64_t)(val & 0x3FFFFFFF);
                                        /*fields[LATITUDE].value = (int32_t)((pure_lat * 50ULL) / 3ULL); 
                                        fields[LATITUDE].available = true; */

                                        setField(fields[LATITUDE], (int32_t)((pure_lat * 50ULL) / 3ULL));          

                                        has_lat = true;
                                        new_lat_received = true; 
                                    } 
                                    else { 
                                        // Negativer Wert -> Längengrad (Longitude / Ost)
                                        uint64_t pure_lon = (uint64_t)(raw_bytes & 0x3FFFFFFF);
                                        /*fields[LONGITUDE].value = (int32_t)((pure_lon * 50ULL) / 3ULL); 
                                        fields[LONGITUDE].available = true; */

                                        setField(fields[LONGITUDE], (int32_t)((pure_lon * 50ULL) / 3ULL)); 

                                        has_lon = true;
                                        new_lon_received = true; 
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

                                                setField(fields[NUMSAT], 12);

                                                live_sat = 12; 
                                                home_is_locked = true; 
                                            }
                                        } 
                                        else {
                                            stable_start_us = current_us;
                                            min_lat = max_lat = fields[LATITUDE].value;
                                            min_lon = max_lon = fields[LONGITUDE].value;
                                        }
                                    }

                                    // BERECHNUNG NUR WENN EIN VOLLES NEUES KOORDINATENPAAR DA IST
                                    if (new_lat_received && new_lon_received) {
                                        
                                        float distance_m = 0.0f;

                                        // Entfernung berechnen (Home Distance)
                                        if (home_lat != 0 && home_lon != 0) {
                                            float dist_lat = (float)(fields[LATITUDE].value - home_lat) * 0.0111139f;
                                            float dist_lon = (float)(fields[LONGITUDE].value - home_lon) * 0.0111139f * 0.6067f; 

                                            distance_m = sqrtf((dist_lat * dist_lat) + (dist_lon * dist_lon));

                                            /*fields[GPS_HOME_DISTANCE].value = (int32_t)distance_m;
                                            fields[GPS_HOME_DISTANCE].available = true;
                                            fields[GPS_HOME_DISTANCE].onceAvailable = true;*/

                                            setField(fields[GPS_HOME_DISTANCE], (int32_t)distance_m);
                                            
                                            fields[NUMSAT].value = 12;
                                            live_sat = 12;
                                        }
                                        

                                        // Kurs BERECHNUNG VOM AUSGANGSPUNKT (HOME) ZUM MODELL
                                        if (home_is_locked && distance_m > 10.0f) {
                                            float dLat = (float)(fields[LATITUDE].value - home_lat);
                                            float dLon = (float)(fields[LONGITUDE].value - home_lon) * 0.6067f;

                                            float angle = atan2f(dLon, dLat);
                                            float heading_deg = angle * 57.29578f;
            
                                            // Schutz vor negativen Floats & Normalisierung auf 0-359.99°
                                            heading_deg = fmodf(heading_deg, 360.0f);
                                            if (heading_deg < 0) heading_deg += 360.0f;

                                            // Runden verhindert negative Vorzeichen-Artefakte beim Abschneiden
                                            int32_t final_heading = (int32_t)roundf(heading_deg * 100.0f);
                                            if (final_heading >= 36000) final_heading = 0;

                                            /*fields[HEADING].value = final_heading;
                                            fields[HEADING].available = true;
                                            fields[HEADING].onceAvailable = true;*/
                                            setField(fields[HEADING], final_heading);

                                        } else {
                                            // 🟢 FIX: Auf dem Tisch oder nahe Home bleibt der Wert jetzt sauber auf 0 stehen
                                            /*fields[HEADING].value = 0;
                                            fields[HEADING].available = true;*/
                                            setField(fields[HEADING], 0);
                                        }

                                        // Flags zurücksetzen für das nächste Paar
                                        new_lat_received = false;
                                        new_lon_received = false;

                                    }    
}

void decode0x820(int32_t val) { // GPS Höhe
    extern int32_t live_alt;
    static int32_t alt_offset = 0; static bool alt_offset_stored = false;
    // Beim allerersten empfangenen Paket merken wir uns die Basishöhe
    if (!alt_offset_stored && val != 0) { alt_offset = val; alt_offset_stored = true; }

    // Ziehe die Basishöhe ab, um die echte relative Höhe (Start bei 0 cm) zu erhalten
    live_alt = val ;  //- alt_offset;
    }   
    
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

void decode0x840(int32_t val) {
   extern int32_t live_sat;
   if (live_sat >= 4) { fields[HEADING].value = val; }// GPS: Kurs (Heading)
    
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
                        switch (id) {

                            case 0x0100:              decode0x100(val)                                           ; break; // Höhe
                            case 0x0110:              setField(fields[VSPEED], val)                              ; break;
                            case 0x0200: case 0x0B60: setField(fields[CURRENT], val * 10); has_current = true    ; break; // FrSky 100mA -> HoTT 10mA
                            case 0x0210:              setField(fields[MVOLT], val * 10)                          ; break;     // FrSky 100mV -> HoTT 10mV
                            case 0x0211:              setField(fields[CAPACITY], val)                            ; break;
                            case 0x0300:              decodeFrSkyCells(val)                                      ; break;
                            case 0x0500:              setField(fields[RPM], val);               has_rpm   = true ; break;    
                            case 0x0511:              setField(fields[GROUNDSPEED], val * 100); has_speed = true ; break;
 
                            case 0x0820:              decode0x820(val)                                           ; break; // GPS Höhe
                            case 0x0830:              decode0x830(val)                                           ; break; // GPS Speed
                            case 0x0840:              decode0x840(val)                                           ; break; // GPS Kurs (Heading)
                            case 0x0800:              decode0x800(val, buffer)                                   ; break; // GPS Koordinaten

                            case 0x0850: // GPS: Uhrzeit, Datum 
                                // Wir prüfen, ob es das Uhrzeit/Sat-Unterpaket ist (niedrigstes Byte == 0x00)
                                    uint8_t raw_byte0 = buffer[4]; // Kennung (0x00 oder 0x01)
                                    uint8_t raw_byte1 = buffer[5];
                                    uint8_t raw_byte2 = buffer[6];
                                    uint8_t raw_byte3 = buffer[7];

                                    break;

                        }


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
            // Das Dauerfeuer für onceAvailable (wird immer beim Auslauf gezündet)
            fields[ALTITUDE].onceAvailable = fields[MVOLT].onceAvailable = fields[CURRENT].onceAvailable = true;
            fields[GROUNDSPEED].onceAvailable = fields[NUMSAT].onceAvailable = fields[RPM].onceAvailable = true;
            fields[PITCH].onceAvailable = fields[ROLL].onceAvailable = fields[HEADING].onceAvailable = true;
            fields[CAPACITY].onceAvailable = fields[TEMP1].onceAvailable = fields[TEMP2].onceAvailable = true;
            fields[RELATIVEALT].onceAvailable = fields[RESERVE2].onceAvailable = fields[VSPEED].onceAvailable = true;
            fields[LATITUDE].onceAvailable = fields[LONGITUDE].onceAvailable = true;
           

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
                if (active_sensors_count > 0 && active_sensors[0] == 0x1B) {
                    current_polling_id = 0x1B; 
                } else {
                    current_polling_id = 0x00; 
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
                setRgbColorOn(0, 1, 0); // 🟢 Ganz kurz matt an
            } else {
                setRgbColorOn(0, 0, 0); // Aus
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
                setRgbColorOn(0, 1, 0); // 🟢 Helles Grün für das Paket!

                if (debugTlm == 'Y') {
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
                setRgbColorOn(0, 0, 0); 

                printAllTelemetryValues();
            } 
            else {
                // Wenn das Fenster abgelaufen ist und KEIN gültiges Paket kam,
                // schalten wir sie hier ebenfalls stur aus, damit sie glimmt.
                setRgbColorOn(0, 0, 0); 
            }

            scheduler_step++;
            if (scheduler_step >= 4) scheduler_step = 0; 
        }
    }
}

void simulate_frsky_sensors_core1() {
    // 🟢 SYSTEMATISCH GETRENNT: Vario (0x1B), Spannung/Strom (0x03), Zellenpaare (0x41..0x44) und GPS (0x22)
    static const uint8_t physical_ids[] = {0x1B, 0x03, 0x41, 0x42, 0x43, 0x44, 0x22}; 
    static uint8_t current_id_idx = 0;
    static uint32_t last_poll_time = 0;
    static bool led_state = false;

    // Statische Variablen für die dynamischen Testwerte
    static int32_t sim_alt = 1000;       
    static int32_t sim_volt = 238;       
    static int32_t sim_current = 150;     
    static int32_t sim_pitch = 0;        
    static uint8_t sim_sats = 8;         
    static int8_t direction = 1;         

    // GPS-Simulationswerte (Hambühren Basis)
    static int32_t sim_lat_frsky = 31585260;  
    static int32_t sim_lon_frsky = 5964732;   
    static int32_t sim_gps_alt = 2420;        
    static int32_t sim_gps_speed = 0;         
    static int32_t sim_gps_heading = 164;     

    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (now - last_poll_time >= 12) {
        last_poll_time = now;
        
        uint8_t target_id = physical_ids[current_id_idx];
        current_id_idx = (current_id_idx + 1) % sizeof(physical_ids);

        // LED-Blitz bei Abfrage
        gpio_put(25, !led_state); 
        sleep_us(100); 
        gpio_put(25, led_state); 

        // Werte verändern
        if (target_id == 0x1B) {
            sim_alt += (50 * direction);       
            sim_volt += (1 * direction);       
            sim_current += (2 * direction);    
            sim_pitch += (1 * direction);      

            sim_lat_frsky += (2 * direction);       
            sim_lon_frsky += (1 * direction);       
            sim_gps_alt += (10 * direction);        
            sim_gps_speed += (55 * direction);      
            sim_gps_heading += (3 * direction);     

            if (sim_gps_speed < 0) sim_gps_speed = 0;
            if (sim_gps_heading > 360) sim_gps_heading = 0;
            if (sim_gps_heading < 0) sim_gps_heading = 360;

            if (sim_alt > 15000 || sim_alt < 0) { 
                direction = -direction;            
                sim_sats = (direction > 0) ? 11 : 6; 
            }
        }

        bool packet_processed = false;
        uint8_t test_frame[12] = {0}; // Puffergröße krisensicher deklariert
        // 1. ZWEIG FÜR DAS VARIO (0x1B)
        if (target_id == 0x1B) {
            static bool toggle_id = false;
            toggle_id = !toggle_id;
            test_frame[0] = 0x7E; test_frame[1] = 0x1B; test_frame[2] = 0x10;
            if (toggle_id) {
                test_frame[3] = 0x00; test_frame[4] = 0x08; // ID 0x0800 (Höhe)
                test_frame[5] = (sim_alt & 0xFF); test_frame[6] = ((sim_alt >> 8) & 0xFF);
                test_frame[7] = ((sim_alt >> 16) & 0xFF); test_frame[8] = ((sim_alt >> 24) & 0xFF);
            } else {
                test_frame[3] = 0x30; test_frame[4] = 0x04; // ID 0x0430 (Pitch)
                test_frame[5] = (sim_pitch & 0xFF); test_frame[6] = ((sim_pitch >> 8) & 0xFF);
            }
            packet_processed = true;
        }
        // 2. ZWEIG FÜR KLASSISCH SPANNUNG & STROM (0x03)
        else if (target_id == 0x03) {
            static bool toggle_v_i = false;
            toggle_v_i = !toggle_v_i;
            test_frame[0] = 0x7E; test_frame[1] = 0x03; test_frame[2] = 0x10;
            if (toggle_v_i) {
                test_frame[3] = 0x10; test_frame[4] = 0x02; // ID 0x0210 (VFAS Gesamtspannung) [INDEX]
                test_frame[5] = (sim_volt & 0xFF); test_frame[6] = ((sim_volt >> 8) & 0xFF);
            } else {
                test_frame[3] = 0x00; test_frame[4] = 0x02; // ID 0x0200 (Strom)
                test_frame[5] = (sim_current & 0xFF); test_frame[6] = ((sim_current >> 8) & 0xFF);
            }
            packet_processed = true;
        }
        // 3. SEPARATE GEHÄUSE FÜR DIE EINZELZELLEN (0x41 bis 0x44)
        else if (target_id == 0x41 || target_id == 0x42 || target_id == 0x43 || target_id == 0x44) {
            test_frame[0] = 0x7E; test_frame[1] = target_id; test_frame[2] = 0x10;
            test_frame[3] = 0x00; test_frame[4] = 0x03; // S.Port Application ID bleibt starr 0x0300 [INDEX]

            uint16_t volt_a = 4100 + (sim_pitch * 2); 
            uint16_t volt_b = 4080 - (sim_pitch * 2); 
            uint16_t bits_a = (volt_a / 2) & 0x0FFF;
            uint16_t bits_b = (volt_b / 2) & 0x0FFF;

            uint8_t packet_sub_id = 0;
            if (target_id == 0x41) packet_sub_id = 0; // Zelle 1 & 2 [INDEX]
            if (target_id == 0x42) packet_sub_id = 2; // Zelle 3 & 4 [INDEX]
            if (target_id == 0x43) packet_sub_id = 4; // Zelle 5 & 6 [INDEX]
            if (target_id == 0x44) packet_sub_id = 6; // Zelle 7 & 8 [INDEX]

            uint32_t val32 = packet_sub_id | ((uint32_t)bits_a << 8) | ((uint32_t)bits_b << 20);
            test_frame[5] = (val32 & 0xFF);         test_frame[6] = ((val32 >> 8) & 0xFF);
            test_frame[7] = ((val32 >> 16) & 0xFF); test_frame[8] = ((val32 >> 24) & 0xFF);
            packet_processed = true;
        }
        // 4. ZWEIG FÜR GPS (0x22)
        else if (target_id == 0x22) {
            static uint8_t gps_rotation_step = 0;
            gps_rotation_step = (gps_rotation_step + 1) % 6;
            test_frame[0] = 0x7E; test_frame[1] = 0x22; test_frame[2] = 0x10;

            switch (gps_rotation_step) {
                case 0: test_frame[3] = 0x00; test_frame[4] = 0x08; 
                        test_frame[5] = (sim_lat_frsky & 0xFF); test_frame[6] = ((sim_lat_frsky >> 8) & 0xFF);
                        test_frame[7] = ((sim_lat_frsky >> 16) & 0xFF); test_frame[8] = ((sim_lat_frsky >> 24) & 0xFF); break;
                case 1: { uint32_t signed_lon = (uint32_t)sim_lon_frsky | 0x80000000;
                        test_frame[3] = 0x00; test_frame[4] = 0x08; 
                        test_frame[5] = (signed_lon & 0xFF); test_frame[6] = ((signed_lon >> 8) & 0xFF);
                        test_frame[7] = ((signed_lon >> 16) & 0xFF); test_frame[8] = ((signed_lon >> 24) & 0xFF); break; }
                case 2: test_frame[3] = 0x20; test_frame[4] = 0x08; 
                        test_frame[5] = (sim_gps_alt & 0xFF); test_frame[6] = ((sim_gps_alt >> 8) & 0xFF);
                        test_frame[7] = ((sim_gps_alt >> 16) & 0xFF); test_frame[8] = ((sim_gps_alt >> 24) & 0xFF); break;
                case 3: test_frame[3] = 0x30; test_frame[4] = 0x08; 
                        test_frame[5] = (sim_gps_speed & 0xFF); test_frame[6] = ((sim_gps_speed >> 8) & 0xFF);
                        test_frame[7] = ((sim_gps_speed >> 16) & 0xFF); test_frame[8] = ((sim_gps_speed >> 24) & 0xFF); break;
                case 4: test_frame[3] = 0x50; test_frame[4] = 0x08; 
                        test_frame[5] = 0x00; test_frame[6] = 0x30; test_frame[7] = sim_sats; test_frame[8] = 0x00; break;
                case 5: { int32_t frsky_heading = sim_gps_heading * 100; 
                        test_frame[3] = 0x40; test_frame[4] = 0x08; 
                        test_frame[5] = (frsky_heading & 0xFF); test_frame[6] = ((frsky_heading >> 8) & 0xFF);
                        test_frame[7] = ((frsky_heading >> 16) & 0xFF); test_frame[8] = ((frsky_heading >> 24) & 0xFF); break; }
            }
            packet_processed = true;
        }

        if (packet_processed) {
            uint32_t crc_calc = 0;
            for (int i = 2; i < 9; i++) crc_calc += test_frame[i];
            while (crc_calc > 0xFF) crc_calc = (crc_calc & 0xFF) + (crc_calc >> 8);
            test_frame[9] = 0xFF - (uint8_t)crc_calc;

            for (int i = 0; i < 10; i++) handleMultiProtocol_N(test_frame[i]);

            fields[RELATIVEALT].value     = fields[ALTITUDE].value;
            fields[RELATIVEALT].available = true; fields[RELATIVEALT].onceAvailable = true;
            fields[RESERVE2].value        = fields[MVOLT].value;
            fields[RESERVE2].available    = true; fields[RESERVE2].onceAvailable = true;
            
            if (now % 1000 < 500) {
                if ((now / 12) % 4 == 0) { setRgbColorOn(0, 1, 0); } else { setRgbColorOn(0, 0, 0); }
            } else { setRgbColorOn(0, 0, 0); }

            static uint32_t last_sim_toggle = 0;
            if (now - last_sim_toggle >= 150) { last_sim_toggle = now; led_state = !led_state; }
        }
    }
}

void simulate_frsky_sensors_core1_f() {
    // 🟢 SYSTEMATISCH GETRENNT: oXs-Erweiterung (0x77) hinten angehängt, Rest unberührt!
    static const uint8_t physical_ids[] = {0x1B, 0x03, 0x41, 0x42, 0x43, 0x44, 0x22, 0x77}; 
    static uint8_t current_id_idx = 0;
    static uint32_t last_poll_time = 0;
    static bool led_state = false;

    // Statische Variablen für die dynamischen Testwerte
    static int32_t sim_alt = 1000;       
    static int32_t sim_volt = 238;       
    static int32_t sim_current = 150;     
    static int32_t sim_pitch = 0;        
    static uint8_t sim_sats = 8;         
    static int8_t direction = 1;         

    // GPS-Simulationswerte (Hambühren Basis)
    static int32_t sim_lat_frsky = 31585260;  
    static int32_t sim_lon_frsky = 5964732;   
    static int32_t sim_gps_alt = 2420;        
    static int32_t sim_gps_speed = 0;         
    static int32_t sim_gps_heading = 164;     

    // NEU & GETRENNT: Simulationswerte für OpenXSensor (oXs)
    static int32_t sim_oxs_alt = 6000;    // Start bei 60.00 Metern (in cm)
    static int32_t sim_oxs_vspd = 250;   // Start bei +2.5 m/s (in cm/s)

    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (now - last_poll_time >= 12) {
        last_poll_time = now;
        
        uint8_t target_id = physical_ids[current_id_idx];
        current_id_idx = (current_id_idx + 1) % sizeof(physical_ids);

        // LED-Blitz bei Abfrage
        gpio_put(25, !led_state); 
        sleep_us(100); 
        gpio_put(25, led_state); 

        // Werte verändern
        if (target_id == 0x1B) {
            sim_alt += (50 * direction);       
            sim_volt += (1 * direction);       
            sim_current += (2 * direction);    
            sim_pitch += (1 * direction);      

            // Getrennte oXs-Berechnung
            sim_oxs_vspd = (180 * direction);
            sim_oxs_alt += (sim_oxs_vspd / 10);

            sim_lat_frsky += (2 * direction);       
            sim_lon_frsky += (1 * direction);       
            sim_gps_alt += (10 * direction);        
            sim_gps_speed += (55 * direction);      
            sim_gps_heading += (3 * direction);     

            if (sim_gps_speed < 0) sim_gps_speed = 0;
            if (sim_gps_heading > 360) sim_gps_heading = 0;
            if (sim_gps_heading < 0) sim_gps_heading = 360;

            if (sim_alt > 15000 || sim_alt < 0) { 
                direction = -direction;            
                sim_sats = (direction > 0) ? 11 : 6; 
            }
        }
        bool packet_processed = false;
        uint8_t test_frame[12] = {0}; // Puffergröße krisensicher deklariert

        // 1. ZWEIG FÜR DAS VARIO (0x1B) - Absolut Original!
        if (target_id == 0x1B) {
            static bool toggle_id = false;
            toggle_id = !toggle_id;
            test_frame[0] = 0x7E; test_frame[1] = 0x1B; test_frame[2] = 0x10;
            if (toggle_id) {
                test_frame[3] = 0x00; test_frame[4] = 0x08; // ID 0x0800 (Höhe)
                test_frame[5] = (sim_alt & 0xFF); test_frame[6] = ((sim_alt >> 8) & 0xFF);
                test_frame[7] = ((sim_alt >> 16) & 0xFF); test_frame[8] = ((sim_alt >> 24) & 0xFF);
            } else {
                test_frame[3] = 0x30; test_frame[4] = 0x04; // ID 0x0430 (Pitch)
                test_frame[5] = (sim_pitch & 0xFF); test_frame[6] = ((sim_pitch >> 8) & 0xFF);
            }
            packet_processed = true;
        }
        // 2. ZWEIG FÜR KLASSISCH SPANNUNG & STROM (0x03) - Absolut Original!
        else if (target_id == 0x03) {
            static bool toggle_v_i = false;
            toggle_v_i = !toggle_v_i;
            test_frame[0] = 0x7E; test_frame[1] = 0x03; test_frame[2] = 0x10;
            if (toggle_v_i) {
                test_frame[3] = 0x10; test_frame[4] = 0x02; // ID 0x0210 (VFAS Gesamtspannung) [INDEX]
                test_frame[5] = (sim_volt & 0xFF); test_frame[6] = ((sim_volt >> 8) & 0xFF);
            } else {
                test_frame[3] = 0x00; test_frame[4] = 0x02; // ID 0x0200 (Strom)
                test_frame[5] = (sim_current & 0xFF); test_frame[6] = ((sim_current >> 8) & 0xFF);
            }
            packet_processed = true;
        }
        // 3. SEPARATE GEHÄUSE FÜR DIE EINZELZELLEN (0x41 bis 0x44) - Absolut Original!
        else if (target_id == 0x41 || target_id == 0x42 || target_id == 0x43 || target_id == 0x44) {
            test_frame[0] = 0x7E; test_frame[1] = target_id; test_frame[2] = 0x10;
            test_frame[3] = 0x00; test_frame[4] = 0x03; // S.Port Application ID bleibt starr 0x0300 [INDEX]

            uint16_t volt_a = 4100 + (sim_pitch * 2); 
            uint16_t volt_b = 4080 - (sim_pitch * 2); 
            uint16_t bits_a = (volt_a / 2) & 0x0FFF;
            uint16_t bits_b = (volt_b / 2) & 0x0FFF;

            uint8_t packet_sub_id = 0;
            if (target_id == 0x41) packet_sub_id = 0; // Zelle 1 & 2 [INDEX]
            if (target_id == 0x42) packet_sub_id = 2; // Zelle 3 & 4 [INDEX]
            if (target_id == 0x43) packet_sub_id = 4; // Zelle 5 & 6 [INDEX]
            if (target_id == 0x44) packet_sub_id = 6; // Zelle 7 & 8 [INDEX]

            uint32_t val32 = packet_sub_id | ((uint32_t)bits_a << 8) | ((uint32_t)bits_b << 20);
            test_frame[5] = (val32 & 0xFF);         test_frame[6] = ((val32 >> 8) & 0xFF);
            test_frame[7] = ((val32 >> 16) & 0xFF); test_frame[8] = ((val32 >> 24) & 0xFF);
            packet_processed = true;
        }
        // 4. ZWEIG FÜR GPS (0x22) - Absolut Original!
        else if (target_id == 0x22) {
            static uint8_t gps_rotation_step = 0;
            gps_rotation_step = (gps_rotation_step + 1) % 6;
            test_frame[0] = 0x7E; test_frame[1] = 0x22; test_frame[2] = 0x10;

            switch (gps_rotation_step) {
                case 0: test_frame[3] = 0x00; test_frame[4] = 0x08; 
                        test_frame[5] = (sim_lat_frsky & 0xFF); test_frame[6] = ((sim_lat_frsky >> 8) & 0xFF);
                        test_frame[7] = ((sim_lat_frsky >> 16) & 0xFF); test_frame[8] = ((sim_lat_frsky >> 24) & 0xFF); break;
                case 1: { uint32_t signed_lon = (uint32_t)sim_lon_frsky | 0x80000000;
                        test_frame[3] = 0x00; test_frame[4] = 0x08; 
                        test_frame[5] = (signed_lon & 0xFF); test_frame[6] = ((signed_lon >> 8) & 0xFF);
                        test_frame[7] = ((signed_lon >> 16) & 0xFF); test_frame[8] = ((signed_lon >> 24) & 0xFF); break; }
                case 2: test_frame[3] = 0x20; test_frame[4] = 0x08; 
                        test_frame[5] = (sim_gps_alt & 0xFF); test_frame[6] = ((sim_gps_alt >> 8) & 0xFF);
                        test_frame[7] = ((sim_gps_alt >> 16) & 0xFF); test_frame[8] = ((sim_gps_alt >> 24) & 0xFF); break;
                case 3: test_frame[3] = 0x30; test_frame[4] = 0x08; 
                        test_frame[5] = (sim_gps_speed & 0xFF); test_frame[6] = ((sim_gps_speed >> 8) & 0xFF);
                        test_frame[7] = ((sim_gps_speed >> 16) & 0xFF); test_frame[8] = ((sim_gps_speed >> 24) & 0xFF); break;
                case 4: test_frame[3] = 0x50; test_frame[4] = 0x08; 
                        test_frame[5] = 0x00; test_frame[6] = 0x30; test_frame[7] = sim_sats; test_frame[8] = 0x00; break;
                case 5: { int32_t frsky_heading = sim_gps_heading * 100; 
                        test_frame[3] = 0x40; test_frame[4] = 0x08; 
                        test_frame[5] = (frsky_heading & 0xFF); test_frame[6] = ((frsky_heading >> 8) & 0xFF);
                        test_frame[7] = ((frsky_heading >> 16) & 0xFF); test_frame[8] = ((frsky_heading >> 24) & 0xFF); break; }
            }
            packet_processed = true;
        }
        // 5. NEUER ISOLIERTER ZWEIG FÜR OPENXSENSOR (0x77)
        else if (target_id == 0x77) {
            static bool toggle_oxs = false;
            toggle_oxs = !toggle_oxs;
            test_frame[0] = 0x7E; test_frame[1] = 0x77; test_frame[2] = 0x10;
            
            if (toggle_oxs) {
                // ID 0x0100 (oXs Baro-Höhe)
                test_frame[3] = 0x00; test_frame[4] = 0x01; 
                test_frame[5] = (sim_oxs_alt & 0xFF);         test_frame[6] = ((sim_oxs_alt >> 8) & 0xFF);
                test_frame[7] = ((sim_oxs_alt >> 16) & 0xFF); test_frame[8] = ((sim_oxs_alt >> 24) & 0xFF);
            } else {
                // ID 0x0110 (oXs VSpd / Variometer)
                test_frame[3] = 0x10; test_frame[4] = 0x01; 
                test_frame[5] = (sim_oxs_vspd & 0xFF);        test_frame[6] = ((sim_oxs_vspd >> 8) & 0xFF);
                test_frame[7] = ((sim_oxs_vspd >> 16) & 0xFF); test_frame[8] = ((sim_oxs_vspd >> 24) & 0xFF);
            }
            packet_processed = true;
        }

        if (packet_processed) {
            // Checksummenberechnung - Absolut Original! [INDEX]
            uint32_t crc_calc = 0;
            for (int i = 2; i < 9; i++) crc_calc += test_frame[i];
            while (crc_calc > 0xFF) crc_calc = (crc_calc & 0xFF) + (crc_calc >> 8);
            test_frame[9] = 0xFF - (uint8_t)crc_calc;

            // Ausgabe - Absolut Original!
            for (int i = 0; i < 10; i++) handleMultiProtocol_N(test_frame[i]);

            fields[RELATIVEALT].value     = fields[ALTITUDE].value;
            fields[RELATIVEALT].available = true; fields[RELATIVEALT].onceAvailable = true;
            fields[RESERVE2].value        = fields[MVOLT].value;
            fields[RESERVE2].available    = true; fields[RESERVE2].onceAvailable = true;
            
            // LED-Code am Ende - Absolut Original!
            if (now % 1000 < 500) {
                if ((now / 12) % 4 == 0) { setRgbColorOn(0, 1, 0); } else { setRgbColorOn(0, 0, 0); }
            } else { setRgbColorOn(0, 0, 0); }

            static uint32_t last_sim_toggle = 0;
            if (now - last_sim_toggle >= 150) { last_sim_toggle = now; led_state = !led_state; }
        }
    }
}
