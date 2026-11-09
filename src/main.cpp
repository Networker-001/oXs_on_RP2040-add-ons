#include "pico/stdlib.h"


#include "stdio.h"
#include "config.h"
#include "hardware/i2c.h"
#include "MS5611.h"
#include "SPL06.h"
#include "BMP280.h"
#include "ms4525.h"
#include "sdp3x.h"
#include "xgzp6897d.h"
#include "vario.h"
#include "voltage.h"
#include "gps.h"
#include "tusb.h"
#include "crsf_in.h"
#include "crsf_out.h"
#include "param.h"
#include "hardware/pio.h"
#include "sbus_out_pwm.h"
#include "sbus_in.h"
#include "hardware/watchdog.h"
#include "tools.h"
#include "sport.h"
#include "jeti.h"
#include "exbus.h"
#include "hott.h"
#include "mpx.h"
#include "ibus.h"
#include "ibus_in.h"
#include "sbus2_tlm.h"
#include "fbus.h"
#include "srxl2.h"
#include "sequencer.h"
//#include "param.h"

#include "ws2812.h"
#include "lps22hb.h"

#include "rpm.h"
#include "EMFButton.h"
#include "ads1115.h"
#include "mpu.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"
#include "ds18b20.h"
#include "hardware/timer.h"
#include "logger.h"
#include "esc.h"
#include "gyro.h"
#include "rfm95.h"
#include "sx126x_driver.h"
#include "kx134.h"
#include "frsky_hub.h"
#include "uart_frsky_master.pio.h" // 🟢 Baut die Brücke zu den mstrens-PIO-Befehlen!
#include "inav_frsky.h"  // 🟢 KORREKTUR: Macht handleMultiProtocol_N in der main.cpp bekannt
#include "inav_ltm.h" // Einbinden des LTM-Parsers für iNav



// to do : add rpm, temp telemetry fields to jeti protocol
//         try to detect MS5611 and other I2C testing the different I2C addresses
//         if ds18b20 would be supported, then change the code in order to avoid long waiting time that should block other tasks.
//         stop core1 when there is no I2C activity while saving the config (to avoid I2C conflict)
//         add airspeed field and compensated Vspeed to all protocols (currently it is only in sport and some other)
//         add spektrum protocol (read the bus already in set up, change baudrate, fill all fields in different frames)

//         test 16 rc channel values in log interface 
//         test logger param in config parameters
//         test tlm data in log interface
//         it seems that in ELRS protocol, PWM are not generated since some version.
//         use Rc channels with gyro correction to the signal Sbus out. 
//         in mpu, when we apply offsets for acc and gyro, we should check that we do not exceed the 16 bits (or put the values in 32 bits)

//         manage the failsafe flag in Sbus out for protocols
//         add AOA telemetry field when GPS, baro and gyro are installed.

//         explain the learning process (for orientation).

// Look at file in folder "doc" for more details
//
// So pio 0 sm0 is used for CRSF Tx  or for Sport TX or JETI TX or HOTT TX or MPX TX or SRXL2 or Ibus (it uses max 6 bytes for Hott and a dma)
//        0   1                         for Sport Rx            or HOTT RX or MPX RX or SRXL2 or Ibus (it uses 9 bytes, 1 irq0 and no dma)
//        0   2            sbus out                                                            (it uses 4 bytes and one dma)       
//        0   3            esc  Rx                                                             (it uses 9 bytes, 1 irq and no dma)

//        1   0 is used for gps Tx  ; is unclaim when GPS config is done and reused             (it uses 4 bytes no dma)      
//        1   0 is also used for gps Rx                                                         (it uses 9 bytes, 1 irq1 and no dma)

//        1   1  is used for rpm                                                                       (it uses 3 bytes , no Irq, no dma)
//        1   2 is used for logger uart Tx                                                             (it uses 4 bytes, no irq and one dma) 
//        1   3 is used for RGB led                                                                    (it uses 4 bytes, no irq and no dma)     
// So UART0 is used for Secondary crsf in or Sbus in ( was GPS before)
//    UART1 is used for primary crsf in or SBUS IN (was only Sbus in before)
 
// note : GPS setup (on core 1) can end when core 1 is already in main loop; setup of logger (pio/dma) has to wait that gps set up is done 

// Pin that can be used are:
// C1 = 0/15  ... C16 = 0/15
// GPS_TX = 0/29
// GPS_RX = 0/29
// PRI = 5 ,9, 21 ,25  (UART1)
// SEC = 1, 13 , 17 ,29 (UART0) 
// SBUS_OUT = 0/29
// TLM = 0/29
// VOLT1= 26/29 ... VOLT4 = 26/29
// SDA = 2, 6, 10, 14, 18, 22, 26  (I2C1)
// SCL = 3, 7, 11, 15, 19, 23, 27  (I2C1)
// SPI_MOSI=
// SPI_MISO=
// SPI_SLCK= 
// SPI_CS= 
// RPM = 0/29 
// LED = 16

// General principle:
// We use the 2 cores: core0 manages in/out signals with receivers, PWM and USB commands, core1 manages all sensors
// In main loop (core0) we:
// - get the value from the sensors (via a queue)
// - send the telemetry data (taking care of the protocol)
// - get the rchannels (in sbusFrame) according to the protocol
// - decide if oXs rc failsafe values must be applied (when no rcchannels are recived within some delay)
// - copy sbusFrame to sbusFrameCorr (if sbusFrame changed)
// - apply gyro corrections on sbusFrameCorr
// - generate sbus out and PWM signal based on sbusFrameCorr
// - manage sequencer
// - manage logger
// - manage the button and led


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



extern queue_t fbusRxQueue; // Sagt dem Compiler, dass die Queue existiert
extern void handleMultiProtocol_N(uint8_t b); // 🟢 KORREKTUR: Zwingt den Compiler, die Funktion zu kennen
extern void read_frsky_sensors_core1(void); // 🟢 KORREKTUR: Zwingt den Compiler, die Funktion zu kennen

void getSensors_ltm(void); // Funktions-Prototyp bekannt machen


// Globales Lipo-Array für die Einzelzellen (Einheit: Millivolt)
//extern uint16_t hott_custom_cells[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//extern bool has_hott_custom_cells = false;

VOLTAGE voltage ;    // class to handle voltages

int32_t i2cError = 0;

MS5611 baro1;    // class to handle MS5611; adress = 0x77 or 0x76
SPL06 baro2;    // class to handle SPL06; adress = 0x77 or 0x76
BMP280 baro3 ;    // class to handle BMP280; adress = 0x77 or 0x76
LPS22HB baro4;    // class to handle LPS22HB; adress = 0x5C (Unser neues Baro!)

ADS1115 adc1( I2C_ADS_Add1 , 0) ;     // class to handle first ads1115 (adr pin connected to grnd)
ADS1115 adc2( I2C_ADS_Add2 , 1) ;     // class to handle second ads1115 (adr pin connected to vdd)

MS4525 ms4525 ( (uint8_t) MS4525_ADDRESS ) ; // 0x28 is the default I2C adress of a 4525DO sensor)
SDP3X sdp3x( (uint8_t) SDPXX_ADDRESS) ;      // 0X21 is the default I2C address of asdp31,... sensor (diffrent for sdp8xx)
XGZP  xgzp( (uint8_t) XGZP_ADDRESS );        // 0x6D is the only one I2C adress

VARIO vario1;

//queue_t gpsQueue ; // queue is used to transfer the data from the uart0 used by GPS
GPS gps;

// objet to manage the mpu6050
MPU mpu(1);
#ifdef KX134_IS_USED
KX134 kx134(1);
#endif

LOGGER logger;

field fields[NUMBER_MAX_IDX];  // list of all telemetry fields and parameters that can be measured (not only used by Sport)

// remapping from sbus value to pwm value
uint16_t fromSbusMin = FROM_SBUS_MIN;
uint16_t toPwmMin = TO_PWM_MIN; 
uint16_t fromSbusMax = FROM_SBUS_MAX;
uint16_t toPwmMax = TO_PWM_MAX; 

// CRSF is managed with 2 pio and not with the internal uart1 in order to freely select the pins

EMFButton btn (3, 0); // button object will be associated to the boot button of rp2040; requires a special function to get the state (see tool.cpp)
                       // parameters are not used with RP2040 boot button 
extern uint32_t lastRcChannels;
extern bool rcChannelsUsChanged ; // says that rcChannelUs changed or not (to avoid some updates) (reset at each main loop)       
extern bool rcChannelsUsCorrChanged ; // says that rcChannelUsCorr changed or not (to avoid some updates) (reset at each main loop)       
extern bool newRcChannelsFrameReceived ;  // used to update the PWM data
extern bool newRcChannelsReceivedForLogger;  // used to know when we have to update the logger data

extern CONFIG config;
bool configIsValid = true;
bool configIsSaved = true ;
bool configIsValidPrev = true;
bool multicoreIsRunning = false;
bool blinking = true ;
uint8_t ledState = STATE_NO_SIGNAL;
uint8_t prevLedState = STATE_NO_SIGNAL;

uint32_t lastBlinkMillis;

extern SEQUENCER seq;
extern struct gyroMixer_t gyroMixer ; // contains the parameters provided by the learning process for each of the 16 Rc channel
extern bool gyroIsInstalled ;

queue_t qSensorData;       // send one sensor data to core0; when type=0XFF, it means a command then data= the command (e.g.0XFFFFFFFF = save config)
queue_t qSendCmdToCore1;
volatile bool core1SetupDone = false;
volatile bool core1OrientationMPUDone = false; // flag filled during the gyro learning process to say that orientation is done
void core1_main(); // prototype of core 1 main function

uint8_t forcedFields = 0; // use to debug a protocol; force the values when = 'P' (positive) or 'N' (negative)

int32_t cameraPitch;
int32_t cameraRoll;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

extern bool calibrateImuGyro ; // recalibrate the gyro or not at reset (avoid it after a watchdog reset)

#define LED_PIN           25 // 25 ist Standard beim Pico. Bei Bedarf anpassen!

#include "pico/util/queue.h"

bool GpsAlt = false; // Flag to indicate if GPS altitude is available

// Wir nutzen die von mstrens initialisierte Rx-Queue
extern queue_t fbusRxQueue; 

void setupI2c(){
    if ( config.pinScl == 255 || config.pinSda == 255) return; // skip if pins are not defined
    // send 10 SCL clock to force sensor to release sda
    /*
    gpio_init(config.pinSda);
    gpio_init(config.pinScl);
    gpio_set_dir(config.pinSda, GPIO_IN);
    gpio_set_dir(config.pinScl, GPIO_OUT);
    gpio_pull_up(config.pinSda);
    gpio_pull_up(config.pinScl);
    sleep_us(10);
    while ( gpio_get(config.pinSda) == 0) {;
        
        for (uint8_t i=0; i<9; i++){
            gpio_put(config.pinScl, 1);
            sleep_us(10);
            gpio_put(config.pinScl, 0);
            sleep_us(10);
            printf("trying to unlock I2C\n");
        }
    }    
    gpio_put(config.pinScl, 1);
    gpio_set_dir(config.pinSda, GPIO_OUT);
    gpio_put(config.pinScl, 0);
    sleep_us(10);
    gpio_put(config.pinSda, 0);
    sleep_us(10);
    gpio_put(config.pinScl, 1);
    sleep_us(10);
    gpio_put(config.pinSda, 1);
    sleep_us(10);
    gpio_set_dir(config.pinSda, GPIO_IN);
    if ( gpio_get(config.pinSda) == 0) printf("I2C still locked\n");    
    */
    // initialize I2C     
    i2c_init( i2c1, 400 * 1000);
    gpio_set_function(config.pinSda, GPIO_FUNC_I2C);
    gpio_set_function(config.pinScl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pinSda);
    gpio_pull_up(config.pinScl); 
}

void setColorState(){    // set the colors based on the RF link
    lastBlinkMillis = millisRp(); // reset the timestamp for blinking
    switch (ledState) {
        case STATE_OK:
            setRgbColorOn(0, 10, 0); //green
            break;
        case STATE_PARTLY_OK:
            setRgbColorOn(10, 5, 0); //yellow
            break;
        case STATE_FAILSAFE:
            setRgbColorOn(0, 0, 10); //blue
            break;
        case STATE_GYRO_CAL_MIXER_NOT_DONE:
            setRgbColorOn(10, 0, 0); //red
            break;
        case STATE_GYRO_CAL_MIXER_DONE:
            setRgbColorOn(10, 5, 0); //yellow
            break;
        case STATE_GYRO_CAL_LIMIT:
            setRgbColorOn(0, 0, 10); //blue
            break;
        default:
            setRgbColorOn(10, 0, 0); //red
            break;     
    }
}

void setupSensors(){     // this runs on core1!!!!!!!!
      
      //sleep_ms(3000);
      //printf("start core1 setup\n");
      //startTimerUs(0); //xxxxx to debug only 
      voltage.begin();      
      //printf("voltage done\n");
      setupI2c();      // setup I2C
      //printf("i2C done\n");
      baro1.begin();  // check MS5611; when ok, baro1.baroInstalled  = true
      //printf("baro1 done\n");
      if (! baro1.baroInstalled) {
        baro2.begin();  // check SPL06;  when ok, baro2.baroInstalled  = true
        //printf("baro2 done\n");
      }
      if (! ( baro1.baroInstalled or baro2.baroInstalled)) {
        baro3.begin(); // check BMP280;  when ok, baro3.baroInstalled  = true
        //printf("baro3 done\n");
      }
      // Waveshare 1.0.2 board has a LPS22HB baro on it; we check it only if no other baro is installed:
      if (! (baro1.baroInstalled or baro2.baroInstalled or baro3.baroInstalled)) {
        baro4.begin(); 
      }

      adc1.begin() ; 
      //printf("adc1 done\n");
      adc2.begin() ;
      //printf("adc2 done\n");
      #ifndef KX134_IS_USED
      mpu.begin();
      #else
      kx134.begin(); 
      #endif
      //printf("mpu done\n");
    //blinkRgb(0,10,0,500,1000000); blink red, green, blue at 500 msec for 1000 0000 X
      gps.setupGps();  //use a Pio and 1 sm (in fact reuse the same sm for RX after TX)
      //printf("gps done\n");
      ms4525.begin();
      if (! ms4525.airspeedInstalled) {
        sdp3x.begin();
        if (! sdp3x.airspeedInstalled) {
            xgzp.begin();
        }
      }
      #ifdef USEDS18B20
      ds18b20Setup(); 
      #endif
      setupRpm(); // this function perform the setup of pio Rpm
      //printf("rpm done\n");
      
      setupEsc() ; 

      core1SetupDone = true;
      //printf("end core1 setup\n") ;    
      //getTimerUs(0);   // xxxxx
}

// read and update all sensor values (runs on core 1)		
void getSensors(void){                                                          // this runs on core1

  
		voltage.getVoltages();                                                   // read voltage sensors
		
		if ( baro1.baroInstalled){                                              // check if baro 1 is present
			if ( baro1.getAltitude() == 0) {                                      // calculate altitude for baro 1
				vario1.calculateAltVspeed(baro1.altitudeCm , baro1.altIntervalMicros ); 
			}
		} else if ( baro2.baroInstalled){                                        // check if baro 2 is present
			if ( baro2.getAltitude() == 0) {                                      // calculate altitude for baro 2
				vario1.calculateAltVspeed(baro2.altitudeCm , baro2.altIntervalMicros); 
			}
		} else if (baro3.baroInstalled) {                                        // check if baro 3 is present
			if ( baro3.getAltitude() == 0) {                                      // calculate altitude for baro 3
				vario1.calculateAltVspeed(baro3.altitudeCm , baro3.altIntervalMicros); 
			}
		} else if (baro4.baroInstalled) {                                        // check if baro 4 (LPS22HB) is present
			if ( baro4.getAltitude() == 0) {                                      // calculate altitude for baro 4
				vario1.calculateAltVspeed(baro4.altitudeCm , baro4.altIntervalMicros); 
			}
		}

		adc1.readSensor();                                                      // read analog sensor 1
		adc2.readSensor();                                                      // read analog sensor 2
		mpu.getAccZWorld();                                                     // read mpu acc Z
		#ifdef KX134_IS_USED
		kx134.getAcc();                                                         // read kx134 acc
		#endif  

		gps.readGps();								                            // Alternativer Standard GPS-Parser

		if (ms4525.airspeedInstalled){                                          // check MS4525 airspeed sensor
			ms4525.getDifPressure();                                              // read differential pressure
			calculateAirspeed( );                                                 // calculate airspeed
			vario1.calculateVspeedDte();                                          // calculate DTE vspeed
		}
		if (sdp3x.airspeedInstalled){                                           // check SDP3X airspeed sensor
			sdp3x.getDifPressure();                                               // read differential pressure
			calculateAirspeed( );                                                 // calculate airspeed
			vario1.calculateVspeedDte();                                          // calculate DTE vspeed
		}
		if (xgzp.airspeedInstalled){                                            // check XGZP airspeed sensor
			xgzp.getDifPressure();                                                // read differential pressure
			calculateAirspeed( );                                                 // calculate airspeed
			vario1.calculateVspeedDte();                                          // calculate DTE vspeed
		}
		readRpm();                                                              // read RPM sensor
		handleEsc();                                                            // process ESC telemetry
		#ifdef USE_DS18B20
		ds18b20Read();                                                          // read DS18B20 temperature sensor
		#endif
  
  
  // Berechnung für den Anstellwinkel (AoA) - läuft für beide Modi, falls die Daten da sind
  if (fields[VSPEED].onceAvailable && fields[GROUNDSPEED].onceAvailable && fields[PITCH].onceAvailable){ 
    float aoa = (float) fields[PITCH].value -  atan2f((float) fields[VSPEED].value, (float) fields[GROUNDSPEED].value) * 180.0 / 3.1416; 
    sent2Core0( RESERVE5 , (int32_t) aoa) ;                                     
  }
}


enum bootButtonStates_t {NOT_ARMED, ARMED, SAVED};
bootButtonStates_t bootButtonState =  NOT_ARMED;

void handleBootButton(){ 
    // check boot button; after double click, change LED to fix blue and next HOLD within 5 sec save the current channnels as Failsafe values
    static uint32_t bootArmedMillis = 0;
    btn.tick();
    if ( ( btn.hasClicks() == 2) && ( (millisRp() - lastRcChannels ) < 100 ) ) { // double click + a recent frame exist
        bootArmedMillis = millisRp();
        bootButtonState =  ARMED;
        setRgbColorOn(0, 0, 10); //blue
        //printf("armed\n");
    } else if (bootButtonState == ARMED) {
        if (btn.hasOnlyHeld() && ( (millisRp() - lastRcChannels ) < 100 )) {     // saved when long hold + recent frame exist
            bootButtonState =  SAVED;
            setRgbColorOn(5, 5, 5);
            bootArmedMillis = millisRp();
            cpyChannelsAndSaveConfig();   // copy the channels values and save them into the config.
            //printf("saving failsafe\n");
        } else if ( (millisRp() - bootArmedMillis) > 5000) {
            bootButtonState =  NOT_ARMED;  // reset if no hold withing the 5 sec
            setColorState();               // restore colors based on RF link
            //printf("loosing armed\n");
        }
    } else if ((bootButtonState == SAVED) && ((millisRp() - bootArmedMillis) > 2000) ){
        bootButtonState =  NOT_ARMED;  // reset after 2 sec
        setColorState();               // restore colors based on RF link 
        //printf("done\n");
    }
}

volatile uint8_t testU8 = 0;
void test_callback(uint alarmNum){
    testU8++;
}


// process incoming sensor data and commands queued from core 1
void getSensorsFromCore1(){
    queue_entry_t entry;
    //printf("qlevel= %d\n",queue_get_level(&qSensorData));

    bool loggerHeaderSent = false;  // used to sent the synchro byte 0X7E only once per function call.
    
    while( !queue_is_empty(&qSensorData)){
        if ( queue_try_remove(&qSensorData,&entry)){
            if (entry.type >= NUMBER_MAX_IDX) {
                if (entry.type == SAVE_CONFIG_ID && entry.data == 0XFFFFFFFF) {  // this is a command to save the config.
                    watchdog_enable(15000,false);
                    sleep_ms(1000);
                    printf("\nCalibration has been done");
                    printConfigOffsets();
                    printf("\nConfig will be saved\n\n");
                    sleep_ms(1000);
                    saveConfig();
                    printf("Config has been saved\n");  
                    printf("Device will reboot\n\n");
                    watchdog_enable(1500,false);
                    sleep_ms(1000);
                    watchdog_reboot(0, 0, 100); // this force a reboot!!!!!!!!!!
                } else if (entry.type == CAMERA_PITCH_ID) {
                    cameraPitch = entry.data;
                } else if (entry.type == CAMERA_ROLL_ID) {
                    cameraRoll = entry.data;
                } else if (entry.type == GYRO_X_ID) {  
                    gyroX = (15*gyroX + entry.data) >> 4;  // here some filtering because oXs use a BW=188 for mpu instead of BW=5 for flightstab in gyro setting.
                    //gyroX = entry.data;
                } else if (entry.type == GYRO_Y_ID) { 
                    gyroY = (15*gyroY + entry.data) >>4 ;
                    //gyroY = entry.data; 
                } else if (entry.type == GYRO_Z_ID) { 
                    gyroZ = (15*gyroZ + entry.data) >> 4;
                    //gyroZ = entry.data;
                } else {
                    printf("error : invalid type of sensor = %d\n", entry.type);
                }    
            } else {
                fields[entry.type].value = entry.data;
                fields[entry.type].available = true ;
                if (fields[entry.type].onceAvailable == false){
                    fields[entry.type].onceAvailable = true ;
                    // update sportMaxBandwidth for some protocols
                    if ( (config.protocol == 'S') || (config.protocol == 'F') )  calculateSportMaxBandwidth(); 
                }    
                if (config.pinLogger != 255) { // when logger is on
                    if ( loggerHeaderSent == false) {  // log once per function call the synchro byte 
                        logger.logByteNoStuff(0X7E);
                        logger.logTimestampMs(millisRp());
                        loggerHeaderSent = true; 
                    }
                    logger.logint32withStuff(entry.type ,entry.data);    // log the type (index) and value of the tlm field
                }                                                     // !!! can wait that previous buffer is totally sent by dma
                //printf("t=%d  %10.0f\n",entry.type ,  (float)entry.data);
            }    
        }
    }
    if ((forcedFields == 1) || (forcedFields == 2)) fillFields(forcedFields); // force dummy values for debuging a protocol
}



/*
void printTest(int testVal){
    static uint32_t prevTime = 0;
    if ((millisRp() - prevTime) > 10) { // once per 5 sec
        prevTime = millisRp();
        printf("test value=%i\n",testVal);
        if (configIsValid) {printf("config is valid\n");} else {printf("config is not valid\n");} 
    }
}
*/

// Am Anfang der Datei (außerhalb der Funktionen) das neue Flag deklarieren:
bool ltm_data_rx_flag = false; 

void handle_ltm_led_core1() {
    static uint32_t last_packet_time = 0;
    uint32_t ms = to_ms_since_boot(get_absolute_time());

    if ((config.inav_ltm == 2)||(config.inav_ltm == 4)  )  {
        static bool pause_eingeleitet = false; // Merker für die Pause

        if (ms % 50 < 10){
            pause_eingeleitet = false; // AN-Phase läuft, Merker zurücksetzen
            
            // Unser erprobtes, mattes Takt-Flimmern für die Augen
            if ((ms / 12) % 10 == 0) {
                setRgbColorOn(0, 0, 10); // 🔵 Mattes Blau-Flackern
                            } else {
                setRgbColorOn(0, 0, 0); 
            }
        } else {
            // AUS-Phase läuft! Wir schalten die LED NUR EINMALIG hart aus:
            if (!pause_eingeleitet) {
                setRgbColorOn(0, 0, 0);    // LED komplett abdunkeln
                pause_eingeleitet = true;  // Sperre setzen, damit der Treiber Ruhe hat!
            }
        }
                // Schritt 2: Die Schranke für die große Halbsekunden-Pause dahinter setzen!
        // In den zweiten 500ms der Sekunde knallen wir die LED unbarmherzig auf Null.
        if (ms % 1000 >= 500) {
            setRgbColorOn(0, 0, 0); // Absolute, digitale Funkstille für eine halbe Sekunde!
        }
    }



    // =========================================================================
    // 🟢 DIE PHYSIKALISCHE RETTUNG FÜR MODUS 1 (INTELLIGENTES SUCH-BLINKEN):
    // Frei von blockierenden Treibersperren. Rechnet stur im sauberen Takt!
    // =========================================================================
    else if (config.inav_ltm == 1 || config.inav_ltm == 3) {
        // Wir prüfen dein originales Parser-Flag
        if (ltm_data_rx_flag == true) {
            last_packet_time = ms;
            ltm_data_rx_flag = false; 
        }

        // ---------------------------------------------------------------------
        // ZUSTAND 1: DATEN SIND DA! (Hektisches, super mattes Blau-Flackern)
        // ---------------------------------------------------------------------
        if (ms - last_packet_time < 300) {
            if (ms % 60 < 30) {
                if ((ms / 12) % 10 == 0) {
                    setRgbColorOn(0, 0, 1); // 🔵 Mattes Blau-Flackern bei Empfang
                } else {
                    setRgbColorOn(0, 1, 0);
                }
            } else {
               // setRgbColorOn(0, 0, 0);
            }
        } 
        // ---------------------------------------------------------------------
        // ZUSTAND 2: KEINE DATEN DA! (Ruhiges Suchen-Blinken im Sekundentakt)
        // Kopiert eins zu eins die Logik aus Modus 2, die bei dir perfekt läuft!
        // ---------------------------------------------------------------------
        else {
            static bool suche_pause_gesendet = false; // Sperr-Merker für die AUS-Phase

            if (ms % 1000 < 500) {
                suche_pause_gesendet = false; // AN-Phase läuft, Merker zurücksetzen
                
                // Exakt dein funktionierendes Flimmer-Raster aus Modus 2
                if ((ms / 12) % 10 == 0) {
                    setRgbColorOn(0, 0, 1); // 🔵 Mattes Blau-Flackern in der AN-Phase
                } else {
                    setRgbColorOn(0, 0, 0); 
                }
            } else {
                // AUS-Phase läuft! Wir senden das "Aus" NUR EIN EINZIGES MAL:
                if (!suche_pause_gesendet) {
                    setRgbColorOn(0, 0, 0);       // LED komplett abdunkeln
                    suche_pause_gesendet = true;  // Sperre setzen, Treiber hat Sendepause!
                }
            }
        }
       // setRgbColorOn(0, 0, 20);
    }
    
}

void getSensors_ltm(void) {

    uint32_t now_ms = millisRp(); // Holt den Millisekunden-Takt von Core 0
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Zeitstempel für das letzte LTM-Paket merken
    static uint32_t last_ltm_packet_ms = 0;

    // =========================================================================
    // MODUS 1: LTM Echtbetrieb über den Hardware-UART
    // =========================================================================
    if (config.inav_ltm == 1)  {

        bool altWasAvailable = fields[ALTITUDE].available;
        bool voltWasAvailable = fields[MVOLT].available;

        parse_inav_ltm(); // Rattert durch deinen LTM Seriell-Parser


    } 
    // =========================================================================
    // MODUS 2: LTM Simulationsbetrieb am Schreibtisch
    // =========================================================================
    else if (config.inav_ltm == 2) {

        simulate_all_sensors_ltm();
    }
    
}



#define MAIN_LOOP 0



// Initialisiert das Gesamtsystem, alle Schnittstellen und Telemetrie-Protokolle
void setup() {
	stdio_init_all();								// Standard-I/O für USB-Debug initialisieren
	set_sys_clock_khz(133000, false);				// Systemtakt auf 133 MHz festlegen

#ifdef DEBUG
	uint16_t counter = 10;							// Warterechner für USB-CDC Verbindung
	while ((!tud_cdc_connected()) && (counter--)) {	// Auf USB-Verbindung warten
		sleep_ms(100);								// Warteraum in Millisekunden
	}
	sleep_ms(2000);									// Wartezeit für Nachrichtenanzeige
	uint8_t a1[2] = {1, 2};							// Testarray für Debug
	int debug = 2;									// Debug-Ebene
	debugAX("aa", a1, 2);							// Debug-Ausgabe
#endif

	if (watchdog_caused_reboot()) {					// Prüfung ob Watchdog-Neustart vorlag
		printf("Rebooted by Watchdog!\n");			// Warnmeldung ausgeben
		calibrateImuGyro = false;					// Kalibrierung bei Reboot überspringen
	} else {
		printf("Clean boot\n");						// Normaler Systemstart
		calibrateImuGyro = true;					// Kalibrierung aktivieren
	}

	setupConfig();									// Konfigurationsparameter laden
	setupSequencers();								// Sequenzerparameter laden
	setupGyroMixer();								// Kreisel-Mischer initialisieren
	checkConfigAndSequencers();						// Konfigurationsprüfung durchführen

    if ((config.inav_ltm == 1)||(config.inav_ltm == 2 ) ) {
	  setupLed();									// Status-LED initialisieren
	  setRgbColorOn(0, 0, 10);						// LED auf Rot schalten während Setup
    }
    else if (config.inav_ltm == 0) {
	  setupLed();									// Status-LED initialisieren
	  setRgbColorOn(10, 0, 0);						// LED auf Blau schalten während Setup
    }

  // Ämderung durch INAV
	if (configIsValid || (config.inav_ltm <= 2)) {								// Setup nur bei gültiger Konfig ausführen


		if (config.pinHigh < 30) {					// Pin High-Level Erzwingung
			gpio_init(config.pinHigh);				// Pin initialisieren
			gpio_set_dir(config.pinHigh, GPIO_OUT);	// Pin als Ausgang setzen
			gpio_put(config.pinHigh, 1);			// Ausgang auf High schalten
		}
		if (config.pinLow < 30) {					// Pin Low-Level Erzwingung
			gpio_init(config.pinLow);				// Pin initialisieren
			gpio_set_dir(config.pinLow, GPIO_OUT);	// Pin als Ausgang setzen
			gpio_put(config.pinLow, 0);				// Ausgang auf Low schalten
		}

		for (uint8_t i = 0; i < NUMBER_MAX_IDX; i++) {	// Liste aller Telemetriefelder leeren
			fields[i].value = 0;					// Telemetriewert zurücksetzen
			fields[i].available = false;			// Verfügbarkeit zurücksetzen
			fields[i].onceAvailable = false;		// Einmalige Verfügbarkeit zurücksetzen
		}

		queue_init(&qSensorData, sizeof(queue_entry_t), 50); // Sensor-Queue für Core 1 initialisieren
		queue_init(&qSendCmdToCore1, 1, 10);		// Befehls-Queue für Core 1 initialisieren
		multicore_launch_core1(core1_main);			// Core 1 starten
		multicoreIsRunning = true;					// Multicore-Status setzen

		uint32_t setup1StartUs = microsRp();		// Zeitmessung Start Core 1
		while (core1SetupDone == false) {			// Warten auf Core 1 Bereitschaft
			if ((microsRp() - setup1StartUs) > (2 * 1000000)) { // Timeout-Prüfung nach 2s
				sleep_ms(3000);						// Pause
				printf("Attention: setup on core 1 did not ended within timeout\n"); // Fehler
				break;								// Schleife bei Timeout verlassen
			}
		}
		uint32_t core1SetupUs = microsRp() - setup1StartUs; // Dauer Core 1 Setup berechnen
		printf("Setup1 takes %d usec\n", (int)core1SetupUs); // Ausführungszeit ausgeben



		// INAV Initialisierung
		if ((config.inav_ltm == 1 ) || (config.inav_ltm == 2 )) { // iNav LTM Telemetrie
			init_inav_ltm();						// LTM UART auf GP4 starten
		} 
		// INAV Initialisierung
		//if (config.inav_ltm == 3 ) { // iNav LTM Telemetrie
		//	init_inav_frsky();						// LTM UART auf GP4 starten
		//}    

        // Empfänger / Telemetrie-Protokoll Initialisierung
        if (config.protocol == 'C') {		// CRSF Protokoll
			setupCrsfIn();							// Primären Rx-UART einrichten
			setupCrsf2In();							// Sekundären Rx-UART einrichten
			setupCrsfOut();							// CRSF TX PIO/DMA einrichten
		} else if (config.protocol == 'S') {		// FrSky S.Port
			setupSbusIn();							// SBUS Input 1 initialisieren
			setupSbus2In();							// SBUS Input 2 initialisieren
			setupSport();							// S.Port Telemetrie initialisieren
		} else if (config.protocol == 'B') {		// Frsky Hub
			setupSbusIn();							// SBUS Input 1 initialisieren
			setupSbus2In();							// SBUS Input 2 initialisieren
			setupFrskyHub();						// FrSky Hub Telemetrie starten
		} else if (config.protocol == 'J') {		// Jeti Telemetrie
			setupSbusIn();							// SBUS Input 1 initialisieren
			setupSbus2In();							// SBUS Input 2 initialisieren
			setupJeti();							// Jeti Schnittstelle starten
		} else if (config.protocol == 'H') {		// HoTT Telemetrie
			setupSbusIn();							// SBUS Input 1 initialisieren
			setupSbus2In();							// SBUS Input 2 initialisieren
			setupHott();							// Graupner HoTT starten
		} else if (config.protocol == 'M') {		// Multiplex Telemetrie
			setupSbusIn();							// SBUS Input 1 initialisieren
			setupSbus2In();							// SBUS Input 2 initialisieren
			setupMpx();								// Multiplex Telemetrie starten
		} else if (config.protocol == 'I') {		// IBUS Telemetrie
			setupIbusIn();							// IBUS Eingangs-UART starten
			setupIbus();							// FlySky IBUS Telemetrie starten
		} else if (config.protocol == '2') {		// Futaba SBUS2
			setupSbusIn();							// SBUS Input 1 initialisieren
			setupSbus2In();							// SBUS Input 2 initialisieren
			setupSbus2Tlm();						// SBUS2 Telemetrie starten
		} else if (config.protocol == 'F') {		// FrSky FBus
			setupFbus();							// FBus Schnittstelle starten
			setupSbus2In();							// Sekundären Eingang starten
		} else if (config.protocol == 'L') {		// Spektrum SRXL2
			setupSrxl2();							// SRXL2 Protokoll starten
		} else if (config.protocol == 'E') {		// Jeti EXBus
			setupExbus();							// EXBus Protokoll starten
		}

		if (config.pinSbusOut != 255) {				// SBUS Ausgangssignal aktiviert?
			setupSbusOutPio();						// SBUS Out via PIO einrichten
		}

		setupPwm();									// PWM Ausgänge initialisieren

		if (config.pinLogger != 255) {				// SD-Logger aktiviert?
			logger.begin();							// SD-Karten-Logger starten
		}

		if ((config.gyroChanControl <= 16) && (mpu.mpuInstalled)) { // Kreisel vorhanden?
			gyroIsInstalled = true;					// Kreisel-Flag auf aktiv setzen
		}

		watchdog_enable(3500, 0);					// Watchdog-Timer auf 3.5s aktivieren
	}

	//setRgbColorOn(10, 0, 0);						// LED auf Rot schalten (No Signal)
}

void loop() {



    // 2. Phase: Nach 4 Sekunden schaltet der normale oXs-Ablauf ein
    if (((configIsValid) and (configIsSaved)) or (config.inav_ltm > 0)) {  // only if config is valid or inav_ltm is active, we run the main loop




      // =================================================================
      // STRATEGISCHE SINGLE-CORE WEICHE
      // =================================================================
      if ((config.inav_ltm == 1) || (config.inav_ltm == 2) ) {
          //getSensors_ltm();     // Verarbeitet Einlesen, Tischtest & Dauerfeuer direkt auf Core 0
          handle_ltm_led_core1();

      } else {
          
          //handle_ltm_led_core1();
      }
      getSensorsFromCore1(); // Holt im Normalmodus die Hardware-Sensoren von Core 1
      printAllTelemetryValues();

      // =================================================================
   

      //mergeSeveralSensors();
      watchdog_update();
      if ( config.protocol == 'C'){   //elrs/crsf
        fillCRSFFrame();  // send telemetry
        handleCrsfIn();
        handleCrsf2In();
      } else if (config.protocol == 'S') {  // sport
        handleSportRxTx();   // send telemetry
        handleSbusIn();
        handleSbus2In();
      } else if (config.protocol == 'B') {  // frsky Hub
        handleFrskyHubFrames();   // send telemetry
        handleSbusIn();
        handleSbus2In();
      } else if (config.protocol == 'J') {  //jeti
        handleJetiTx();      // send telemetry
        handleSbusIn();
        handleSbus2In();
      } else if (config.protocol == 'H') {  //Hott
        handleHottRxTx();     // send telemetry
        handleSbusIn();
        handleSbus2In();
      } else if (config.protocol == 'M') {  // multiplex
        handleMpxRxTx();       // send telemetry
        handleSbusIn();
        handleSbus2In();
      } else if (config.protocol == 'I') {  // Ibus flysky
        handleIbusRxTx();
        handleIbusIn();           
        //handleSbus2In();
      } else if (config.protocol == '2') { // Sbus2 Futaba
        handleSbusIn();
        handleSbus2In();
      } else if (config.protocol == 'F') {  // Fbus frsky
        handleFbusRxTx();
        handleSbus2In();
      } else if (config.protocol == 'L') {  // SRXL2 Spektrum
        handleSrxl2RxTx();
        //handleSbus2In();  // to do processa second inpunt
      } else if (config.protocol == 'E') {  // Jeti Exbus
        handleExbusRxTx();
        //handleSbus2In();  // to do process a second inpunt
      }
      watchdog_update();
        // apply failsafe values on sbusframe if rchannel frames are missing more that X msec
        // convert sbusframe in rcChannelUs (when a new sbusframe is received or at least once every 9 msec) and copy to rcChannelUsCorr (so ready to apply gyro corrections)
      // rcChannelsUs is used by logger and sequencer; rcChannelCorrUs is used by Gyro, PWM and SbusOut
      setRcChannels();         // coded in sbus_out_pwm.cpp, set rcChannelsUsCorrChanged and rcChannelsUsChanged 
      if (gyroIsInstalled) {   // coded in Gyro.cpp
        calibrateGyroMixers();   // check if user ask for gyro calibration
        applyGyroCorrections();  // apply corrections (if rcChannelsCorrUs changed) 
      }
      //setLedState();                // set the color of the led
      fillSbusFrame(); // once per 9 msec convert current rcChannelsUsCorr back to Sbus format and generate the sbus frame
      updatePWM(); // update PWM pins only based on channel values with corrections (not sequencer); 
      rcChannelsUsCorrChanged = false;  // reset the flag used to avoid updates at each loop
      rcChannelsUsChanged = false;
      newRcChannelsFrameReceived = false;

      sequencerLoop();  // update PWM pins based on sequencer
      if ((config.pinLogger != 255) && (newRcChannelsReceivedForLogger)) { // when logger is on and new RC data have been converted in uint16
        newRcChannelsReceivedForLogger = false; // reset the flag allowing a log of RC channels
        logger.logAllRcChannels();  // log all rc channels
      }    
      
       
  }
  //alarmTimerUs(MAIN_LOOP, 1000);    //  print a warning if enlapsed time exceed xx usec

 
  watchdog_update();
  
  handleUSBCmd();  // MUSS laufen, um den USB-Buffer leerzulesen!
  tud_task();      // Taktet den USB-Stack


  if ( configIsValidPrev != configIsValid) {
    configIsValidPrev = configIsValid;
 /*   if ((configIsValid) and (configIsSaved) and (config.inav_ltm == 0)){
        blinking = true; // setRgbColorOn(0,10,0); // red , green , blue
    } else {
        blinking = false; // setRgbColorOn(10,0,0);
        setRgbOn();  
    }*/
    
   

  }



  //printf("after tud-task\n");sleep_ms(100); 
  /*if ( configIsValidPrev != configIsValid) {
    configIsValidPrev = configIsValid;
    if ((configIsValid) and (configIsSaved)){
       //blinking = true; // setRgbColorOn(0,10,0); // red , green , blue
    } else {
        blinking = false; // setRgbColorOn(10,0,0);
        // setRgbOn();  
    }
  }*/



  //printf("before handleBootButton\n");sleep_ms(100); 
  handleBootButton(); // check boot button; after double click, change LED to fix blue and next HOLD within 5 sec save the current channnels as Failsafe values
  //printf("after handleBootButton\n");sleep_ms(100); 
  if (( bootButtonState == ARMED) || ( bootButtonState == SAVED)){
   // setRgbColorOn(0, 0, 10); //blue
 } else if ( ledState != prevLedState ){
    if (config.inav_ltm == 0) { // 🟢 NUR im normalen oXs-Modus schalten!
        prevLedState = ledState;
        setColorState();     
    }
  } else if ( blinking && (( millisRp() - lastBlinkMillis) > 300 ) ){
    // toggleRgb();
    lastBlinkMillis = millisRp();
  }
  //if (get_bootsel_button()) {
  //  printf("p\n");
  //} 
  //enlapsedTime(0);
  //printf("end of loop\n");sleep_ms(100);   
 
}

int main_o(){
  setup();
  while(1) loop();
}

int main() {
    setup();
    
    while(1) {
        // Solange wir in den ersten 4 Sekunden sind, überspringen wir den loop()
       /* if (millisRp() < 4000) {
            watchdog_update(); // Verhindert den Hardware-Reset des RP2040
            waitUs(1000);      // 1 Millisekunde Pause im Leerlauf
            continue;          // Springt sofort wieder an den Anfang der while(1)
        }*/
        
        // Erst nach 4 Sekunden wird der normale oXs-Loop ausgeführt
        loop();
    }
}


//======================================================================================

// initialisation of core 1 that capture the sensor data
void setup1(){
    multicore_lockout_victim_init();
    setupSensors();    
    
    if ((config.inav_ltm == 3)||(config.inav_ltm == 4 ) ) {
	  setupLed();									// Status-LED initialisieren
	  setRgbColorOn(2, 0, 0);						// LED auf Grün schalten während Setup
    }

    if (config.inav_ltm >= 3 ) { // iNav LTM Telemetrie
        init_frsky_sensors_core1(); 
        //init_inav_frsky();       // LTM UART auf GP4 starten
    }  
}


// main loop on core 1 in order to read the sensors and send the data to core0
#define MAIN_LOOP1 1

void loop1() {
    //startTimerUs(MAIN_LOOP1);
    uint8_t qCmd;

    // Wenn inav_ltm == 0 ist, läuft die originale oXs-Sensorverarbeitung
    if (config.inav_ltm == config.inav_ltm) {          //0

        getSensors(); // get sensor

        if (config.pinSpiCs != 255) {
            loraHandle();
        }

        // Holt Kalibrierungsbefehle von Core 0 ab
        if (!queue_is_empty(&qSendCmdToCore1)) {
            queue_try_remove(&qSendCmdToCore1, &qCmd);
            if (qCmd == REQUEST_USB_HORIZONTAL_MPU_CALIB) { 
                mpu.usbOrientationHorizontalExecute();
            } else if (qCmd == REQUEST_USB_VERTICAL_MPU_CALIB) { 
                mpu.usbOrientationVerticalExecute();
            } else if (qCmd == REQUEST_NEXT_ACC_CALIB) {
                mpu.nextAccCalibrationExecute();
            } else if (qCmd == REQUEST_HORIZONTAL_MPU_ORIENTATION) {
                mpu.orientationExecute(true);
            } else if (qCmd == REQUEST_VERTICAL_MPU_ORIENTATION) {
                mpu.orientationExecute(false); 
            } else if (qCmd == REQUEST_GYRO_CALIBRATION) {
                mpu.gyroCalibrationExecute(); 
            } 
        }
    } // <-- Hier schließt mstrens das obere "if (config.inav_ltm == 0)"

    //  MODUS 1 & 2: Altes LTM-Live und LTM-Tischtest (Übergibt die LED-Arbeit an Core 1)
    else if (config.inav_ltm == 1)  {parse_inav_ltm();}
       // handle_ltm_led_core1();

    else if(config.inav_ltm == 2) { simulate_all_sensors_ltm();}
    
    // Wenn inav_ltm == 3 ist, läuft dein FrSky-Sensor-Modus völlig exklusiv
    else if(config.inav_ltm == 3) {read_frsky_sensors_core1();    }

    // NEU - TESTMODUS: Künstliche Daten ohne Sensor erzeugen
    else if(config.inav_ltm == 4) {simulate_frsky_sensors_core1();}

    
    //alarmTimerUs(MAIN_LOOP1, 500);
}


void core1_main() {
    setup1();
    
    while(1) {
        // Solange wir in den ersten 4 Sekunden sind, überspringen wir loop1()
        /*if (millisRp() < 4000) {
            // Hier erlauben wir Core 1 im Hintergrund sicher zu atmen,
            // ohne dass er die zeitkritischen Sensor-Loops triggert.
            waitUs(1000); 
            continue; // Springt sofort wieder an den Anfang der while(1)
        }*/
        
        // Erst nach 4 Sekunden starten die Core-1-Sensoren und Tasks
        loop1();
    }
}


void core1_main_o(){
    setup1();
    while(1) loop1();
}


