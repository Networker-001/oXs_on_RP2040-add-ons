#include <stdio.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "inav_ltm.h"
#include "param.h"
#include "tools.h"
#include <stdint.h>
#include <stdbool.h>
#include "ws2812.h"
// 🟢 KORREKTUR: Wir binden die echten oXs-Core-Header ein
#include "param.h"
#include "tools.h"
#define INAV_UART uart1
#define INAV_BAUDRATE 19200

// =================================================================
// ADAPTION: GLOBALE SPEICHER-SCHUBLADEN DIESES LTM-SENSOR-MODULS
// =================================================================
int32_t live_alt = 0; // Startwert 1 Meter für HoTT-Init
int32_t live_alt_rel = 0;
int32_t live_mvolt = 0;
int32_t live_current = 0;
int32_t live_speed = 0;
int32_t live_vspeed = 0;
int32_t live_sat = 0;
int32_t live_rpm = 0;
int32_t live_cap = 0;
int32_t live_pitch = 0;
int32_t live_roll = 0;
int32_t live_heading = 0;
uint8_t live_flight_mode = 0;
uint8_t live_is_armed    = 0;    
int32_t live_lat = 0;
int32_t live_lon = 0;


bool has_mvolt = false;
bool has_current = false;
bool has_speed = false;
bool has_sat = false;
bool has_rpm = false;
bool has_cap = false;
bool has_alt = false;
bool has_alt_rel = false;
bool has_alt_uni = false;
bool has_lat = false;
bool has_lon = false;
bool has_pitch = false;
bool has_roll = false;
bool has_head = false;

//#define DEBUGINAV
//#define DEBUGINAV3
extern uint8_t debugTlm;
extern uint8_t debugVal;
extern uint8_t debugAll;
extern VARIO vario1;

extern field fields[];

extern bool ltm_data_rx_flag;

// Initialisiert die UART-Schnittstelle fuer den nativen iNav LTM-Empfang (Normal/Nicht-Invertiert!)
void init_inav_ltm() {
    uart_init(INAV_UART, INAV_BAUDRATE);                      // UART mit 19200 Baud starten
    uart_set_hw_flow(INAV_UART, false, false);                // Kein Hardware-Flow-Control nutzen
    uart_set_format(INAV_UART, 8, 1, UART_PARITY_NONE);       // Format 8N1 einstellen
    
    // Pin GP5 als RX zuweisen (Dein physisches Kabel von TX4 der Drohne)
    gpio_set_function(5, GPIO_FUNC_UART);                     
}

// Das neue Makro nutzt den Pointer-Trick, um den Index im fields-Array zu bestimmen
#define setField(f, v) do { \
    sent2Core0((&(f) - fields), (int32_t)(v)); \
    if (debugVal == 'Y') { \
        printf("\r %-25s | Value: %8ld        ", #f, (int32_t)(v)); \
        fflush(stdout); \
    } \
} while(0)



void parse_inav_ltm() {
    static int      state = 0;                                
    static char     frame_type = 0;                           
    static uint8_t  buffer[30]; // Ein echtes, stabiles Array mit 30 Zellen!
    static int      idx = 0;                                  
    static int      expected_len = 0;                         
    static uint8_t  crc = 0;                                  
     uint32_t now_us = time_us_32();
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    static uint8_t current_polling_id = 0; 

    static uint32_t last_packet_ms = now_ms;
    ltm_data_rx_flag = false;
        // =====================================================================
        // 🟢 DER ULTRA-DUNKEL-TRICK FÜR DEN LEERLAUF:
        // Wir lassen die Stufe 1 nur noch alle 120ms (Modulo 10) ganz kurz 
        // aufblitzen, den Rest der Zeit ist sie aus (0, 0, 0). 
        // Für das Auge ergibt das ein perfekt gedimmtes, mattes Glimmen!
        // =====================================================================
        /*if (now_ms - last_packet_ms >= 200) {
            if ((now_ms / 12) % 50 == 0) {
                setRgbColorOn(0, 0, 1); // 🟢 Ganz kurz matt an
            } else {
                setRgbColorOn(0, 0, 0); // Aus
            }
        } */

    while (uart_is_readable(INAV_UART)) {
        uint8_t c = uart_getc(INAV_UART);

        switch (state) {
            case 0: // Suche nach Start-Byte '$'
                if (c == '$') {
                    state = 1;                                
                }
                break;

            case 1: // Suche nach Header-Byte 'T'
                if (c == 'T') {
                    state = 2;                                
                } else {
                    state = 0;                                
                }
                break;

            case 2: // Bestimmung des Frame-Typs und Laden der REAL vermessenen Laengen!
                frame_type = c;
                crc = 0; // KORREKTUR: CRC startet bei 0, Typ fliegt aus der XOR-Rechnung raus!                                      
                idx = 0;

                if      (frame_type == 'G') expected_len = 15; // 14 Daten + 1 CRC
                else if (frame_type == 'A') expected_len = 7;  // 6 Daten + 1 CRC
                else if (frame_type == 'S') expected_len = 8;  // 7 Daten + 1 CRC
                else if (frame_type == 'O') expected_len = 15; // 14 Daten + 1 CRC
                else if (frame_type == 'N') expected_len = 7;  // 6 Daten + 1 CRC
                else if (frame_type == 'X') expected_len = 7;  // 6 Daten + 1 CRC
                else {
                    state = 0;                                
                    break;
                }
                state = 3;
                break;

            case 3: // Payload stumpf im RAM sammeln
                if (idx < expected_len) {
                    buffer[idx] = c;
                    idx++;
                    // XOR-CRC ueber alle Datenbytes berechnen (außer dem allerletzten CRC-Byte)
                    if (idx < expected_len) {
                        crc ^= c;
                    }                                 
                }
                
                if (idx == expected_len) {
                    state = 4;                                
                }
                break;

            case 4: // Checksumme verifizieren und Daten in den oXs-Topf uebernehmen
                uint8_t received_crc = buffer[expected_len - 1];

                if (crc == received_crc) {                               
                // =============================================================
                // 🟢 HELLES BLITZEN IN GRÜN BEI ANTWORT:
                // Ein echtes Paket kam durch! Wir reißen die LED voll auf (Stufe 20).
                // =============================================================
                 /* last_packet_ms = now_ms;
                  setRgbColorOn(0, 0, 1); // 🟢 Helles Blau für das Paket! */      
                
                    // ==========================================
                    // G-FRAME: GPS Information (15 Bytes)
                    // ==========================================
                    if (frame_type == 'G') {
                        int32_t lat = (int32_t)( (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24) );
                        int32_t lon = (int32_t)( (uint32_t)buffer[4] | ((uint32_t)buffer[5] << 8) | ((uint32_t)buffer[6] << 16) | ((uint32_t)buffer[7] << 24) );
                        uint8_t ground_speed = buffer[8];
                        int32_t gps_alt = (int32_t)( (uint32_t)buffer[9] | ((uint32_t)buffer[10] << 8) | ((uint32_t)buffer[11] << 16) | ((uint32_t)buffer[12] << 24) );
                        uint8_t sat_info = buffer[13];
                        
                        /*fields[LATITUDE].value    = lat;
                        fields[LONGITUDE].value   = lon;
                        fields[GROUNDSPEED].value = ground_speed * 100; 
                        fields[ALTITUDE].value    = gps_alt * 100; // Echte Höhe ohne 1-Meter-Trick    
                        fields[NUMSAT].value      = sat_info >> 2;

                        fields[LATITUDE].available    = true;
                        fields[LONGITUDE].available   = true;
                        fields[GROUNDSPEED].available = true;
                        fields[ALTITUDE].available    = true;
                        fields[NUMSAT].available      = true;*/

                        setField(fields[LATITUDE], lat); 
                        setField(fields[LONGITUDE], lon); 
                        setField(fields[GROUNDSPEED], ground_speed * 100); 
                        setField(fields[ALTITUDE], gps_alt * 100);
                        setField(fields[RELATIVEALT], gps_alt * 100);
                        setField(fields[NUMSAT], sat_info >> 2);
                    }
                    
                    // ==========================================
                    // A-FRAME: Attitude Lage (7 Bytes)
                    // ==========================================
                    else if (frame_type == 'A') {
                        int16_t pitch   = (int16_t)(buffer[0] | (buffer[1] << 8));
                        int16_t roll    = (int16_t)(buffer[2] | (buffer[3] << 8));
                        int16_t heading = (int16_t)(buffer[4] | (buffer[5] << 8));

                        /*fields[PITCH].value   = pitch;
                        fields[ROLL].value    = roll;
                        fields[HEADING].value = heading;

                        fields[PITCH].available   = true;
                        fields[ROLL].available    = true;
                        fields[HEADING].available = true;*/

                       setField(fields[PITCH], pitch);
                       setField(fields[ROLL], roll);
                       setField(fields[HEADING], heading);

                        
                    }
                    
                     // ==========================================
                    // S-FRAME: Status / Telemetrie (8 Bytes)
                    // ==========================================
                    else if (frame_type == 'S') {
                        uint16_t voltage    = buffer[0] | (buffer[1] << 8);
                        uint16_t current    = buffer[2] | (buffer[3] << 8);
                        uint16_t capacity   = buffer[4] | (buffer[5] << 8);
                        uint8_t status_byte = buffer[6]; 

                        // Extraktion der Einzelzustände (Flightmode, Armed, Failsafe)
                        uint8_t flight_mode = status_byte >> 2; 
                        uint8_t is_armed    = status_byte & 0x01;        
                        uint8_t is_failsafe = (status_byte >> 1) & 0x01; 

                        /*fields[MVOLT].value    = voltage; 
                        fields[CURRENT].value  = current * 10; 
                        fields[CAPACITY].value = capacity;
                        fields[TEMP1].value    = flight_mode;            // Reiner iNav-Modus (z.B. 13 fuer RTH)
                        fields[TEMP2].value    = is_armed;               // 1 = Armed, 0 = Disarmed

                        // Failsafe-Sonderbehandlung fuer den Sender
                        if (is_failsafe) {
                            fields[TEMP2].value = 99;                    // 99 signalisiert Failsafe auf der Jeti
                        }

                        fields[MVOLT].available    = true;
                        fields[CURRENT].available  = true;
                        fields[CAPACITY].available = true;
                        fields[TEMP1].available    = true; 
                        fields[TEMP2].available    = true; */

                        setField(fields[MVOLT], voltage);
                        setField(fields[CURRENT], current * 10);
                        setField(fields[CAPACITY], capacity);
                        setField(fields[TEMP1], flight_mode);
                         if (is_failsafe) {
                            setField(fields[TEMP2], 99); // Failsafe signalisiert
                        } else {
                            setField(fields[TEMP2], is_armed);
                        }

                    }

                    // ==========================================
                    // X-FRAME: Extra Daten Airspeed (7 Bytes)
                    // ==========================================
                    else if (frame_type == 'X') {
                        uint16_t ltm_airspeed = buffer[2] | (buffer[3] << 8); 
                        
                        /*fields[AIRSPEED].value     = ltm_airspeed; 
                        fields[AIRSPEED].available = true;*/
                        
                        setField(fields[AIRSPEED], ltm_airspeed);


                    }
                    // =========================================================
                    // 🟢 DIE RETTUNG FÜR MODUS 1: Das Flag zünden!
                    // Sobald IRGENDEIN Frame die CRC-Prüfung fehlerfrei 
                    // passiert hat, schalten wir dein LED-Flag sofort scharf.
                    // =========================================================
                    ltm_data_rx_flag = true;     

                    // =============================================================
                    // 🟢 DER RÜCKSPRUNG: Sobald die Daten verarbeitet sind, 
                    // schalten wir SOFORT wieder aus. Das erzeugt das perfekte Blitzen!
                    // =============================================================
                   // setRgbColorOn(0, 0, 0); 
                    
                }
                state = 0;                                    
                break;
        }
    }
}


void simulate_all_sensors_ltm() {
    static uint32_t last_poll_time = 0;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    extern int32_t live_alt, live_mvolt, live_current, live_speed, live_vspeed, live_sat, live_cap, live_pitch, live_roll, live_heading;
    extern uint8_t live_flight_mode, live_is_armed;

        live_alt         = 15000;    
        live_mvolt       = 12400;    
        live_current     = 1500;     
        live_speed       = 4500;     
        live_sat         = 9;        
        live_rpm         = 3200;     
        live_pitch       = 10;       
        live_roll        = 5;        
        live_heading     = 180;      
        live_cap         = 850;  
        live_flight_mode = 13;
        live_is_armed    = 1;
        live_lat         = 471234567; // 47.1234567° N
        live_lon         = 850123456; // 85.0123456° E
        live_vspeed       = 1200; // 12 m/s

        setField(fields[ALTITUDE], live_alt); 
        setField(fields[RELATIVEALT], live_alt); 
        setField(fields[MVOLT], live_mvolt);
        setField(fields[CURRENT], live_current);
        setField(fields[GROUNDSPEED], live_speed);
        setField(fields[NUMSAT], live_sat);
        setField(fields[RPM], live_rpm);
        setField(fields[PITCH], live_pitch);
        setField(fields[ROLL], live_roll);
        setField(fields[HEADING], live_heading);
        setField(fields[CAPACITY], live_cap);
        setField(fields[TEMP1], live_flight_mode);
        setField(fields[TEMP2], live_is_armed);
        setField(fields[RESERVE2], live_mvolt); 
        setField(fields[VSPEED], live_speed);


    // --- 3. HARDWARE-BLINKTAKTSYNC ---
    //if ((now / 1500) % 2 == 0) { setRgbColorOn(0, 0, 1); } else { setRgbColorOn(0, 0, 0); }

}

void parse_inav_ltm_OK() {
    static int      state = 0;                                
    static char     frame_type = 0;                           
    static uint8_t  buffer[30]; // Ein echtes, stabiles Array mit 30 Zellen!
    static int      idx = 0;                                  
    static int      expected_len = 0;                         
    static uint8_t  crc = 0;                                  

    while (uart_is_readable(INAV_UART)) {
        uint8_t c = uart_getc(INAV_UART);

        switch (state) {
            case 0: // Suche nach Start-Byte '$'
                if (c == '$') {
                    state = 1;                                
                }
                break;

            case 1: // Suche nach Header-Byte 'T'
                if (c == 'T') {
                    state = 2;                                
                } else {
                    state = 0;                                
                }
                break;

            case 2: // Bestimmung des Frame-Typs und Laden der REAL vermessenen Laengen!
                frame_type = c;
                crc = 0; // KORREKTUR: CRC startet bei 0, Typ fliegt aus der XOR-Rechnung raus!                                      
                idx = 0;

                if      (frame_type == 'G') expected_len = 15; // 14 Daten + 1 CRC
                else if (frame_type == 'A') expected_len = 7;  // 6 Daten + 1 CRC
                else if (frame_type == 'S') expected_len = 8;  // 7 Daten + 1 CRC
                else if (frame_type == 'O') expected_len = 15; // 14 Daten + 1 CRC
                else if (frame_type == 'N') expected_len = 7;  // 6 Daten + 1 CRC
                else if (frame_type == 'X') expected_len = 7;  // 6 Daten + 1 CRC
                else {
                    state = 0;                                
                    break;
                }
                state = 3;
                break;

            case 3: // Payload stumpf im RAM sammeln
                if (idx < expected_len) {
                    buffer[idx] = c;
                    idx++;
                    // XOR-CRC ueber alle Datenbytes berechnen (außer dem allerletzten CRC-Byte)
                    if (idx < expected_len) {
                        crc ^= c;
                    }                                 
                }
                
                if (idx == expected_len) {
                    state = 4;                                
                }
                break;

            case 4: // Checksumme verifizieren und Daten in den oXs-Topf uebernehmen
                uint8_t received_crc = buffer[expected_len - 1];

                if (crc == received_crc) {                               
                    
                    // ==========================================
                    // G-FRAME: GPS Information (15 Bytes)
                    // ==========================================
                    if (frame_type == 'G') {
                        int32_t lat = (int32_t)( (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24) );
                        int32_t lon = (int32_t)( (uint32_t)buffer[4] | ((uint32_t)buffer[5] << 8) | ((uint32_t)buffer[6] << 16) | ((uint32_t)buffer[7] << 24) );
                        uint8_t ground_speed = buffer[8];
                        int32_t gps_alt = (int32_t)( (uint32_t)buffer[9] | ((uint32_t)buffer[10] << 8) | ((uint32_t)buffer[11] << 16) | ((uint32_t)buffer[12] << 24) );
                        uint8_t sat_info = buffer[13];
                        
                        fields[LATITUDE].value    = lat;
                        fields[LONGITUDE].value   = lon;
                        fields[GROUNDSPEED].value = ground_speed * 100; 
                        fields[ALTITUDE].value    = gps_alt * 100; // Echte Höhe ohne 1-Meter-Trick    
                        fields[NUMSAT].value      = sat_info >> 2;

                        fields[LATITUDE].available    = true;
                        fields[LONGITUDE].available   = true;
                        fields[GROUNDSPEED].available = true;
                        fields[ALTITUDE].available    = true;
                        fields[NUMSAT].available      = true;
                    }
                    
                    // ==========================================
                    // A-FRAME: Attitude Lage (7 Bytes)
                    // ==========================================
                    else if (frame_type == 'A') {
                        int16_t pitch   = (int16_t)(buffer[0] | (buffer[1] << 8));
                        int16_t roll    = (int16_t)(buffer[2] | (buffer[3] << 8));
                        int16_t heading = (int16_t)(buffer[4] | (buffer[5] << 8));

                        fields[PITCH].value   = pitch;
                        fields[ROLL].value    = roll;
                        fields[HEADING].value = heading;

                        fields[PITCH].available   = true;
                        fields[ROLL].available    = true;
                        fields[HEADING].available = true;
                    }
                    
                     // ==========================================
                    // S-FRAME: Status / Telemetrie (8 Bytes)
                    // ==========================================
                    else if (frame_type == 'S') {
                        uint16_t voltage    = buffer[0] | (buffer[1] << 8);
                        uint16_t current    = buffer[2] | (buffer[3] << 8);
                        uint16_t capacity   = buffer[4] | (buffer[5] << 8);
                        uint8_t status_byte = buffer[6]; 

                        // Extraktion der Einzelzustände (Flightmode, Armed, Failsafe)
                        uint8_t flight_mode = status_byte >> 2; 
                        uint8_t is_armed    = status_byte & 0x01;        
                        uint8_t is_failsafe = (status_byte >> 1) & 0x01; 

                        fields[MVOLT].value    = voltage; 
                        fields[CURRENT].value  = current * 10; 
                        fields[CAPACITY].value = capacity;
                        fields[TEMP1].value    = flight_mode;            // Reiner iNav-Modus (z.B. 13 fuer RTH)
                        fields[TEMP2].value    = is_armed;               // 1 = Armed, 0 = Disarmed

                        // Failsafe-Sonderbehandlung fuer den Sender
                        if (is_failsafe) {
                            fields[TEMP2].value = 99;                    // 99 signalisiert Failsafe auf der Jeti
                        }

                        fields[MVOLT].available    = true;
                        fields[CURRENT].available  = true;
                        fields[CAPACITY].available = true;
                        fields[TEMP1].available    = true; 
                        fields[TEMP2].available    = true; 
                    }

                    // ==========================================
                    // X-FRAME: Extra Daten Airspeed (7 Bytes)
                    // ==========================================
                    else if (frame_type == 'X') {
                        uint16_t ltm_airspeed = buffer[2] | (buffer[3] << 8); 
                        
                        fields[AIRSPEED].value     = ltm_airspeed; 
                        fields[AIRSPEED].available = true;
                    }
                                    // =========================================================
                    // 🟢 DIE RETTUNG FÜR MODUS 1: Das Flag zünden!
                    // Sobald IRGENDEIN Frame die CRC-Prüfung fehlerfrei 
                    // passiert hat, schalten wir dein LED-Flag sofort scharf.
                    // =========================================================
                    ltm_data_rx_flag = true;     
                }
                state = 0;                                    
                break;
        }
    }
}


void getSensors_ltm_o(void) {
    // Sagt dem Compiler, dass diese Variablen in der inav_ltm.cpp liegen
    extern int32_t live_alt;
    extern int32_t live_mvolt;
    extern int32_t live_current;
    extern int32_t live_speed;
    extern int32_t live_sat;
    extern int32_t live_cap;
    extern int32_t live_pitch;
    extern int32_t live_roll;
    extern int32_t live_heading;
    extern uint8_t live_flight_mode;
    extern uint8_t live_is_armed;

    static bool ledInit = false;
    /*if (!ledInit) {
        gpio_init(25);
        gpio_set_dir(25, GPIO_OUT);
        ledInit = true;
    }*/

    // =================================================================
    // SCHRITT 1: DIE WERTE HOLEN
    // =================================================================
    if (config.inav_ltm == 1)  {
        live_alt = 100; 
        live_mvolt = 0;
        live_current = 0;
        live_speed = 0;
        live_sat = 0;
        live_rpm = 0;
        live_cap = 0;
        live_pitch = 0;
        live_roll = 0;
        live_heading = 0;

        has_alt = false;
        has_mvolt = false;
        has_current = false;
        has_speed = false;
        has_sat = false;
        has_rpm = false;
        has_cap = false;

        bool altWasAvailable = fields[ALTITUDE].available;
        bool voltWasAvailable = fields[MVOLT].available;

        parse_inav_ltm();

        if (fields[ALTITUDE].available)    { live_alt = fields[ALTITUDE].value; has_alt = true; }
        if (fields[MVOLT].available)       { live_mvolt = fields[MVOLT].value; has_mvolt = true; }
        if (fields[CURRENT].available)     { live_current = fields[CURRENT].value; has_current = true; }
        if (fields[GROUNDSPEED].available) { live_speed = fields[GROUNDSPEED].value; has_speed = true; }
        if (fields[NUMSAT].available)      { live_sat = fields[NUMSAT].value; has_sat = true; }
        if (fields[RPM].available)         { live_rpm = fields[RPM].value; has_rpm = true; }
        if (fields[CAPACITY].available)    { live_cap = fields[CAPACITY].value; has_cap = true; }
        if (fields[PITCH].available)       { live_pitch = fields[PITCH].value; }
        if (fields[ROLL].available)        { live_roll = fields[ROLL].value; }
        if (fields[HEADING].available)     { live_heading = fields[HEADING].value; }
        if (fields[TEMP1].available)       { live_flight_mode = fields[TEMP1].value; }
        if (fields[TEMP2].available)       { live_is_armed = fields[TEMP2].value; }

        if (fields[ALTITUDE].available == altWasAvailable && fields[MVOLT].available == voltWasAvailable) {
            gpio_put(25, (millisRp() % 1000 < 500)); 
        } else {
            gpio_put(25, (millisRp() % 60 < 30)); 
        }
    } 
    else if (config.inav_ltm == 2) {
        live_alt         = 15000;    
        live_mvolt       = 12400;    
        live_current     = 1500;     
        live_speed       = 4500;     
        live_sat         = 9;        
        live_rpm         = 3200;     
        live_pitch       = 10;       
        live_roll        = 5;        
        live_heading     = 180;      
        live_cap         = 850;  
        live_flight_mode = 13;
        live_is_armed    = 1;
    }
    else if (config.inav_ltm == 3 || config.inav_ltm == 4) {
        // Holt sich die Werte von Core 1 ab
        live_alt         = fields[ALTITUDE].value;       
        live_mvolt       = fields[MVOLT].value;    
        live_current     = fields[CURRENT].value;     
        live_speed       = fields[GROUNDSPEED].value;     
        live_sat         = fields[NUMSAT].value;        
        live_rpm         = fields[RPM].value;     
        live_pitch       = fields[PITCH].value;       
        live_roll        = fields[ROLL].value;        
        live_heading     = fields[HEADING].value;      
        live_cap         = fields[CAPACITY].value;      
        live_flight_mode = fields[TEMP1].value;
        live_is_armed    = fields[TEMP2].value;
    } // <-- HIER WIRD DER FR-SKY BLOCK JETZT SAUBER GESCHLOSSEN!

    // =================================================================
    // SCHRITT 1B: DIE GLOBALE SPIEGELUNG FÜR ALLE MODI (FÜR HOTT/JETI)
    // =================================================================
    fields[ALTITUDE].value     = live_alt;       
    if (config.inav_ltm != 3 && config.inav_ltm != 4) {
        // Macht das Spiegeln nur für Modus 1 und 2, um FrSky nicht zu stören
        fields[MVOLT].value        = live_mvolt;    
        fields[CURRENT].value      = live_current;     
        fields[GROUNDSPEED].value  = live_speed;     
        fields[NUMSAT].value       = live_sat;        
        fields[RPM].value          = live_rpm;     
        fields[PITCH].value        = live_pitch;       
        fields[ROLL].value         = live_roll;        
        fields[HEADING].value      = live_heading;      
        fields[CAPACITY].value     = live_cap;      
    }
    fields[TEMP1].value        = live_flight_mode;
    fields[TEMP2].value        = live_is_armed;

    // =================================================================
    // SCHRITT 2: DAS DAUERFEUER DER FLAGS ERZWINGEN (Für alle Modi!)
    // =================================================================
    fields[ALTITUDE].available    = true; 
    fields[MVOLT].available       = true;
    fields[CURRENT].available     = true; 
    fields[GROUNDSPEED].available = true;
    fields[NUMSAT].available      = true; 
    fields[RPM].available         = true;
    fields[PITCH].available       = true; 
    fields[ROLL].available        = true;
    fields[HEADING].available     = true; 
    fields[CAPACITY].available    = true;
    fields[TEMP1].available       = true; 
    fields[TEMP2].available       = true;

    fields[ALTITUDE].onceAvailable    = true;
    fields[MVOLT].onceAvailable       = true;
    fields[CURRENT].onceAvailable     = true;
    fields[GROUNDSPEED].onceAvailable = true;
    fields[NUMSAT].onceAvailable      = true;
    fields[PITCH].onceAvailable       = true;
    fields[ROLL].onceAvailable        = true;
    fields[HEADING].onceAvailable     = true;
    fields[CAPACITY].onceAvailable    = true;
    fields[TEMP1].onceAvailable       = true;
    fields[TEMP2].onceAvailable       = true;
    
    // Vario-Berechnung synchron halten
    vario1.calculateAltVspeed(fields[ALTITUDE].value, 200000);
    fields[VSPEED].available = true;

    // HoTT-Pflichtfeld-Spiegelung für das GAM-Modul
    fields[RELATIVEALT].value     = fields[ALTITUDE].value;
    fields[RELATIVEALT].available = true;
    fields[RESERVE2].value        = fields[MVOLT].value;
    fields[RESERVE2].available    = true;

    fields[VSPEED].value = 0;
    fields[VSPEED].available = true;
    fields[VSPEED].onceAvailable = true; 


}

