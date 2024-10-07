//This includes the necessary header files
#include "dw3000.h"
#include "SPI.h"
#include <WiFi.h>

#include <BasicLinearAlgebra.h>
#include <math.h>

extern SPISettings _fastSPI;
#define PIN_RST 27
//#define PIN_RST 9
#define PIN_IRQ 34
//#define PIN_IRQ 8
#define PIN_SS 4
//#define PIN_SS 10
// #define PIN_RST  34  // reset pin
// #define PIN_IRQ  35  // irq pin
// #define PIN_SS  5    // spi select pin
//Constant defenitions
//Delays of rx and tx
#define RNG_DELAY_MS 100 //What is the RNG delay? is between the ranging exchanges?
#define TX_ANT_DLY 16391 //TX antenna delay
#define RX_ANT_DLY 16391 //RX antenna delay
//Is this the length of the message
#define ALL_MSG_COMMON_LEN 10 // common length of all messages
#define ALL_MSG_SN_IDX 2 //inde
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4 //length of timestamp in response message
#define POLL_TX_TO_RESP_RX_DLY_UUS 240 //Delay between poll TX and rrespons RX... what is the POLL TX
#define RESP_RX_TIMEOUT_UUS 400

/* Default communication configuration. We use default non-STS DW mode. */
//This is the default configuration for DW3000 chip... Where are the configurations found and are there any modifications?
static dwt_config_t config = {
  5,                /* Channel number. */
  DWT_PLEN_64,      /* Preamble length. Used in TX only. */
  DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
  9,                /* TX preamble code. Used in TX only. */
  9,                /* RX preamble code. Used in RX only. */
  1,                /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
  DWT_BR_6M8,       /* Data rate. */
  DWT_PHRMODE_STD,  /* PHY header mode. */
  DWT_PHRRATE_STD,  /* PHY header rate. */
  //what is the SFD timeout
  (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
  DWT_STS_MODE_OFF, /* STS disabled */
  DWT_STS_LEN_64,   /* STS length see allowed values in Enum dwt_sts_lengths_e */
  DWT_PDOA_M0       /* PDOA mode off */
};

//Static variables
static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[20];
static uint32_t status_reg = 0;
static double tof;
static double distance;
extern dwt_txconfig_t txconfig_options;
float ranges[3];

// Replace with your network credentials
const char* ssid = "Phone";
const char* password = "pondomay";
WiFiServer server(23); // Port 23 for Telnet or custom

// Define the maximum number of sensors
const int MAX_SENSORS = 4;  // Adjust this value as needed

struct Position
{
    float x_pos;
    float y_pos;
    float z_pos;
};

void Wifi_init(){  

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi.");
  Serial.println(WiFi.localIP());
  server.begin(); // Start the server

}


void DWT_Setup(){
  //What is this loop doing?
  while (!dwt_checkidlerc())  { // Need to make sure DW IC is in IDLE_RC before proceeding
    UART_puts("IDLE FAILED\r\n");
    while (1)
      ;
  }
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    UART_puts("INIT FAILED\r\n");
    while (1) // is this just a infinite loop if the initialization fails?
      ;
  }

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config))  // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1)
      ;
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set expected response's delay and timeout. See NOTE 1 and 5 below.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
   * Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

  Serial.println("Range RX");
  Serial.println("Setup over........");
}

void spi_setup(){
  spiBegin(PIN_IRQ, PIN_RST); //Initialize SPI with DW3000 pins
  spiSelect(PIN_SS);
  _fastSPI = SPISettings(16000000L, MSBFIRST, SPI_MODE0); // Configuration SPI settings
  delay(2);  // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
}

void setup() {
  //Wifi stuff
  Serial.begin(115200);


  Wifi_init();

  UART_init(); //Initializing UART

  spi_setup();
  
  DWT_Setup();
  
  
 
  

}


char tagID;
#define TAG_nb 4  //number of TAG used for Ranging
static double dist_result[TAG_nb];

void loop() {
  Serial.println("Entering main loop");
  int iterations = 100;
  float tolerance = .0001;
  // Sensor Setup
  BLA::Matrix<MAX_SENSORS,3> SEN_POS {0,  0, 15,  //Sensor 1 Position
                                      20, 20, 15, //Sensor 2 Position
                                      20,  0, 20, //Sensor 3 Position
                                      }; //Sensor 4 Position


  // safe bet is to just leave it at 0,0,0
  BLA::Matrix<3> INIT_GUESS = {0,0,0};   
  Serial.println("Find distance 1");
  Multi_Ranging('1', 0, 0);
  Serial.println("");
   Serial.println("Find distance 2");
  Multi_Ranging('2', 1, 1);
   Serial.println("Find distance 3");
  Multi_Ranging('3', 2, 2);
  Position Pos;
  WiFiClient client = server.available(); // Check for incoming clients
  while (1) {
    // get three distance measurements
    Multi_Ranging('1', 0, 0);
    Multi_Ranging('2', 1, 1);
    Multi_Ranging('3', 2, 2);
    BLA::Matrix<MAX_SENSORS> DIST_MEAS = {ranges[0], ranges[1], ranges[2]};

    Pos = findPos(SEN_POS, 
                  DIST_MEAS, 
                  INIT_GUESS,
                  iterations,
                  tolerance);

    float POSx = Pos.x_pos;
    float POSy = Pos.y_pos;
    float POSz = Pos.z_pos;

    Serial.println(POSx, 5);
    Serial.println(POSy, 5);
    Serial.println(POSz, 5);
 
    String message = String(int(ranges[0] * 100)) + ", " + String(int(ranges[1] * 100)) + ", " + String(int(ranges[2] * 100)) + ", " +
                            String(int(POSx * 1000)) + ", " + String(int(POSy * 1000)) + ", " + String(int(POSz * 1000)) + ", " + String(millis() + 1000000);
   

    client.println(message);
    Sleep(RNG_DELAY_MS);
  }
  Sleep(RNG_DELAY_MS);
  
}




// Function to calculate the norm of a 3x1 vector
float vectorNorm(const BLA::Matrix<3, 1> &vec) {
    // Calculate the Euclidean (L2) norm
    float norm = sqrt(pow(vec(0, 0), 2) + pow(vec(1, 0), 2) + pow(vec(2, 0), 2));
    return norm;
}

// Function to find positon based on distance measurements
Position findPos(BLA::Matrix<MAX_SENSORS,3> SEN_POS, 
              BLA::Matrix<MAX_SENSORS> &DIST_M, 
              BLA::Matrix<3> &INIT_GUESS,
              int iterations,
              float tolerance) {
    /**
    Estimate the object's position using gradient descent.
    
    Parameters:
    - sensor_positions:   array of shape (m, 3)
    - measured_distances:  array of shape (m,)
    - initial_guess:  array of shape (3,)
    - max_iterations: int, maximum number of iterations
    - tolerance: float, convergence threshold
    
    Returns:
    - estimated_position: array of shape (3,), estimated object position
  **/
 
  BLA::Matrix<MAX_SENSORS,3> A;
  BLA::Matrix<MAX_SENSORS> DIST_EST;

  BLA::Matrix<3> x = INIT_GUESS; // current position estimate
  BLA::Matrix<3> x_new;
  BLA::Matrix<3> s_i; // Sensor position
  float r_i; // measured distance
  float p_i; // = || x - s_i ||  current estimate distance
  float d_i; // = r_i - p_i residual betweem estimated and measured distance
  float tol_check;
  BLA::Matrix<3> a_i; // direction vector = (x-s_i)/ p_i
  
  BLA::Matrix<3, MAX_SENSORS> AT;
  BLA::Matrix<3, 3> ATA;
  BLA::Matrix<3, 3> ATA_INV;
  BLA::Matrix<3, MAX_SENSORS> A_PLUS;
  BLA::Matrix<3> estimated_pos_delta; 

  
  // iteratively solve until tolerance is reached 
  for (int i = 0; i < iterations; i++) {
    Serial.println(i);
    //Build A and d matrix one sensor at a time

    for (int i = 0; i < MAX_SENSORS; i++) {
      
      // extract sensor position and dsitance measurements
      s_i(0) = SEN_POS(i, 0); // 1x3 matrix 
      s_i(1) = SEN_POS(i, 1); // 1x3 matrix 
      s_i(2) = SEN_POS(i, 2); // 1x3 matrix 
      
      r_i = DIST_M(i); // scalar
      

      // compute estimated distance between position and sensor
      p_i = vectorNorm(x-s_i); 
      
      // compute residual distance betweenn measurment and estimate
      d_i = r_i - p_i;

      
 
      a_i = (x - s_i)/p_i;

      A(i, 0) = a_i(0);
      A(i, 1) = a_i(1);
      A(i, 2) = a_i(2);

      DIST_EST(i) = d_i;
    }
    //find pseudoinverse 
    // compute transpose of A
    AT = ~A;
    ATA = AT * A;
    ATA_INV = Inverse(ATA);

    A_PLUS = ATA_INV * AT;

    estimated_pos_delta = A_PLUS * DIST_EST; 

    x_new = x + estimated_pos_delta;

    Serial.print("x_new: ");
    Serial.println(x_new);
    
    tol_check = vectorNorm(x_new - x);

    if (tol_check < tolerance) {
      
      return {x(0), x(1), x(2)};
      
    }
    x = x_new;
  }   

  return {x(0), x(1), x(2)};
}



//process to perform ranging process for given TAG ID
void Multi_Ranging(char tagID, int id, uint8_t dist) {
  double tof; //time of flight
  double distance; //distance from each tag

  //initialize messages for TX and RX
  uint8_t tx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'T', 'A', 'G', tagID, 0xE0, 0, 0 };
  uint8_t rx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'G', tagID, 'T', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  //Set sequence number in TX poll message
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  //Write a 32-bit IC regiser that is part of a sub-addressed block.
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  //Write the TX message data into the device's internal TX buffer
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
  //Configure the TX frame control register
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
  //Initiates transmission of the frame.
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  //DWT_START_TX_IMMEDIATE --> The transmitter starts sending frame immediately
  //DWT_RESPONSE_EXPECTED --> Response is expected, once the frame is sent the transiceiver will enter recieve mode to wait for response message

  //Wait until RX frame is recieved or timeout occurs
  //Function is used to read 32-bit IC registers
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {};

  frame_seq_nb++; //incrmenet of frame sequence number

  //Process RX frame if recieved succesfully
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    uint32_t frame_len;

    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer)) {
      dwt_readrxdata(rx_buffer, frame_len, 0); //Reading the RX data
      //Reads a number, len bytes from the IC receive data buffer, beginning at the speecified offset, bufferOffset, into the given buffer
      // This resets the sequence number in RX buffer
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      //check if recieved message matches expected response
      if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio; //what are these definitions for?

        //Returns the low 32-bits of the 40-bit 
        poll_tx_ts = dwt_readtxtimestamplo32();//readtimestamps... Assuming this is used for calculating distance
        resp_rx_ts = dwt_readrxtimestamplo32();

        //caclucate clock offset ratio
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

        //Extract timestamps from response message
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;
        
        //calcualte time of fligt and distance
        tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        distance = tof * SPEED_OF_LIGHT;
        dist_result[dist] = distance; //Store distance result in array

        ranges[id] = dist_result[dist];
      }
    }
  } else {
    /* Clear RX error/timeout events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  }
  
}

