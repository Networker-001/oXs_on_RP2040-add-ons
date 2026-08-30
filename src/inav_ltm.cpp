#include <stdio.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "inav_ltm.h"
#include "param.h"
#include "tools.h"

#define INAV_UART uart1
#define INAV_BAUDRATE 19200

// =================================================================
// ADAPTION: GLOBALE SPEICHER-SCHUBLADEN DIESES LTM-SENSOR-MODULS
// =================================================================
int32_t live_alt = 100; // Startwert 1 Meter für HoTT-Init
int32_t live_mvolt = 0;
int32_t live_current = 0;
int32_t live_speed = 0;
int32_t live_sat = 0;
int32_t live_rpm = 0;
int32_t live_cap = 0;
int32_t live_pitch = 0;
int32_t live_roll = 0;
int32_t live_heading = 0;
uint8_t live_flight_mode = 0;
uint8_t live_is_armed    = 0;    

bool has_mvolt = false;
bool has_current = false;
bool has_speed = false;
bool has_sat = false;
bool has_rpm = false;
bool has_cap = false;
bool has_alt = false;


extern field fields[];

// Initialisiert die UART-Schnittstelle fuer den nativen iNav LTM-Empfang (Normal/Nicht-Invertiert!)
void init_inav_ltm() {
    uart_init(INAV_UART, INAV_BAUDRATE);                      // UART mit 19200 Baud starten
    uart_set_hw_flow(INAV_UART, false, false);                // Kein Hardware-Flow-Control nutzen
    uart_set_format(INAV_UART, 8, 1, UART_PARITY_NONE);       // Format 8N1 einstellen
    
    // Pin GP5 als RX zuweisen (Dein physisches Kabel von TX4 der Drohne)
    gpio_set_function(5, GPIO_FUNC_UART);                     
}



void parse_inav_ltm() {
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
                }
                state = 0;                                    
                break;
        }
    }
}
