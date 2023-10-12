//--------------------------------------- Libaries -----------------------------------
// General utility
#include <Wire.h>
#include <SPI.h>

// BNO055 IMU, Docs: https://github.com/adafruit/Adafruit_BNO055, https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
#include <Adafruit_BNO055.h>

// MPL3115A2 Altimeter, Docs: https://github.com/adafruit/Adafruit_MPL3115A2_Library, https://cdn-shop.adafruit.com/datasheets/1893_datasheet.pdf
#include <Adafruit_MPL3115A2.h>

// SD Card, Docs: https://github.com/arduino-libraries/SD
#include <SD.h>

// RFM95C LoRa Radio, Docs: https://github.com/adafruit/RadioHead, https://cdn.sparkfun.com/assets/1/5/d/6/6/RFM95CW_Specification_EN_V1.0.pdf?_gl=1*6l4ae4*_ga*MjExMjQ5MDk3OC4xNjg2MTk5NDg1*_ga_T369JS7J9N*MTY5MTQ3OTAxNC41LjAuMTY5MTQ3OTAxNC42MC4wLjA.
#include <RHReliableDatagram.h>
#include <RH_RF95.h>

//---------------------------------- Definitions and Const Variables ---------------------------------

// BNO055 Info
#define BNO_ADDR        0x28  // I2C address of first BNO
#define PAGE_ID         0x07  // BNO register: page select
#define ACC_DATA_X_LSB  0x08  // BNO page 0 register: Acceleration Data X LSB
#define ACC_CONFIG      0x08  // BNO page 1 register: Accelerometer Config
#define MAG_CONFIG      0x09  // BNO page 1 register: Magnetometer Config
#define GYR_CONFIG_0    0x0A  // BNO page 1 register: Gyroscope Config 0
#define MODE_AMG        0x07  // Non-fusion mode with accel/gyro/mag
#define MODE_CONFIG     0x10  // BNO page 0 register: Configuration Mode; Can write to registers in this mode
#define MODE_NDOF       0x0C  // BNO page 0 register: NDOF fusion mode; uses all 3 sensors for absolute orientation     

// MPL3115A2 Info
#define MPL_ADDR        0x60  //I2C address of mpl
#define CTRL_REG1       0x26  //First Control Register

// SD Card Info and Data Logging
const int chipSelect = 4;
const uint8_t numElements = 18;
const uint8_t runningDataSize = 6; //Index 0 used for landing

// RFM95C LoRa Radio Info
#define CLIENT_ADDRESS 1 // Messages are particularily addressed, so defining who is who
#define SERVER_ADDRESS 2
#define RFM95_CS    4  // Pin definitions
#define RFM95_INT   7
#define RFM95_RST   2 
#define RF95_FREQ 915.0 // Default is 434.0 MHz

const int loop_delay_ms = 100;

//----------------------------- Function Declarations and Global Variables -----------------------------

// General 
uint8_t state = 0;
long timeLanded = 0;
long loopTime = 0;

// SD Card Info and Data Logging
void sd_setup();
void save_data();
void get_data();
void cycle_array(float altitude, float accMagnitude);
File flightData;
float all_data[numElements] = {};

// BNO055 Info
void bno_setup();
void i2c_write(uint8_t i2c_addr, uint8_t reg, uint8_t data);
void bno_change_range(uint8_t measurement, uint8_t range);
void bno_change_mode(uint8_t mode);
Adafruit_BNO055 bno;
float accels[runningDataSize] = {};

// MPL3115A2 Info
void mpl_setup();
void update_altitudes();
Adafruit_MPL3115A2 mpl;
float altitudes[runningDataSize] = {-1, -1, -1};

// RFM95C LoRa Radio Info
RH_RF95 driver(RFM95_CS, RFM95_INT); // Singleton instance of the radio driver
RHReliableDatagram manager(driver, CLIENT_ADDRESS); // Class to manage message delivery and receipt, using the driver declared above
uint16_t packetNum = 0;

//----------------------------------------- Setup ---------------------------------------------------
void setup(void){

  Serial.begin(115200);        // initialize serial port
  //while(!Serial) {delay(10);}  // wait for serial port to open, takes around like 1.5 s
  delay(1500);
  Wire.begin();                // initialize I2C
  delay(1000);
  
  sd_setup();
  
  bno_setup();
  
  mpl_setup();
  
  get_data(); //To flush out any starting 0s
  Serial.println(F("Beginnign"));
}
//----------------------------------------- Loop ------------------------------------------------
void loop(void){    
    // states that we can be in: ground, initial boost, coast, apogee, drogue, main, landing
    // We probably wont mess around with these too much cause the data is so noisy, and we might go with a kalman filter to do it anyways
    // altimeter is slowing everything down. faster output with less oversampling ris so irratic that we need to choose a smaller output rate
  if(millis()-loopTime >= loop_delay_ms){
    get_data(); //loopTime is reset everytime in get_data(), right after the new data is gathered
//    double agg = millis();
    save_data();
    cycle_array(all_data[3], all_data[13]);
    switch(state){
      case 0: //Still on ground, sensing for initial boost
        if(all_data[13] >= 9.81*2 || (altitudes[1]-altitudes[2]) >= 3){ //for alt can do velocity, but would just need seom more variables a and a rotating thing
          //Extremely sensitive-> assuming payload is turned on already on pad
          state = 1;
        }
        break;
      case 1: //In initial boost, sensing for coast
        // I think Coast starts slightly after when accel reverses/velocity peaks, pressure continues to decrease, altitude continues to increase
        if(all_data[13] <= 5){ // yikes, gives us a tenth of a second
          state = 2;
        }
        break;
      case 2: //In coase, sensing for apogee
        //apogee is when altitude peaks, spped should reverse, 
      case 3: //In post-apogee, sensing for drogue
      case 4: //Under drogue, sensing for main
      case 5: //Under main, sensing for landing 
        static uint8_t longerCheck = 0; // heh just trying out static vars to see if it makes it more readable
        if(altitudes[0] && accels[0]){  // gyro under certain margin maybe?
          longerCheck++;
          if(longerCheck >= 25){
            state = 7;
            timeLanded = millis();
          }
        }else{
          longerCheck = 0;
        }
        break;
      case 7: //Landed, start post-landing procedures
        if(!altitudes[0] || !accels[0]){
          longerCheck = 0;
          state = 5;
        }else{
          //in futrure, slow camera to pictures?, send trasmission? increase delay?
          if(bno.getMode() != MODE_NDOF){
            bno_change_mode(MODE_NDOF);
            delay(1500); //So we do need this but since we have already landed, i guess this isn't too bad  
          }
          state = 8;
        }
        break;
      case 8: //Idk after a lot of time after landing?
        if(!altitudes[0] || !accels[0]){
          longerCheck = 0;
          state = 5;  
        }else if ((millis()-timeLanded) > 10000){
            flightData.println("End");
            flightData.close();
            delay(1000);
            while(1);
        }
        break;
    }

    //Serial.println(millis()-agg);
  }
}

//--------------------------------------- Functions --------------------------------------------
//                                     ==== Setup ====
void sd_setup(){
  int fileIteration = 0;
  boolean fileCreated = false;
  
  while(!SD.begin(chipSelect)) {
    Serial.println(F("No SD Card detected. Trying again in 2.5 seconds"));
    delay(2500);
  }
  
  while(!fileCreated){
    String fileName = "FLIGHT" + String(fileIteration) + ".csv";
    if(!SD.exists(fileName)){
      flightData = SD.open(fileName, FILE_WRITE);
      fileCreated = true;
      delay(500);// idk, might need a delay for it to open, prob not cause the afore SD.open doesnt need one
    }
    fileIteration++;
  }
  flightData.println(F("Time,state,bnoTemp,altitude,acc.x,acc.y,acc.z,gyr.x,gyr.y,gyr.z,mag.x,mag.y,mag.z,magnitudeAcc,quat.w,quat.x, quat.y,quat.z"));
}

void mpl_setup(){
  mpl = Adafruit_MPL3115A2();
  while(!mpl.begin()) { // Automatically starts in altitude mode
    Serial.println(F("No MPL3115A2 detected. Trying again in 2.5 seconds"));
    delay(2500);
  }
  delay(1000); //Page 6 of datasheet  
  i2c_write(MPL_ADDR, CTRL_REG1, 0x98); //Sacrifice oversample ratio/niose reduction for output speed
  mpl.setSeaPressure(1018); // STD SLP (sea level pressure) = 1013.26 hPa
  //mpl.setAltitudeOffset(int8_t (from -127 to 128 m))
}

// Just creating a new bno object and setting initial parameters like ext crystal use and the initial operating mode
void bno_setup(){
  bno = Adafruit_BNO055(-1, BNO_ADDR, &Wire); //sensor id, i2c address, wire object
  while(!bno.begin(MODE_AMG)){
    Serial.println(F("No BNO055 detected. Trying again in 2.5 seconds"));
    delay(2500);
  }
  bno.setExtCrystalUse(true); // might be useless if we just change to config mode tho
  delay(750); // Switching to ext crystal can mess things up. Need delay
  bno_change_range(ACC_CONFIG, 0x0F);
  bno_change_range(GYR_CONFIG_0, 0x00); // Pretty sure this is the default, but just make sure
  bno_change_range(MAG_CONFIG, 0x07); // idk if we even really want to use MAG at all
}

//                                ==== Data Reading, Processing, and Saving ====
void get_data(){
  //double agg = millis();
  float altitude = mpl.getAltitude();
  float currentTime = millis()/1000.0;
  //Serial.println(millis()-agg);
  float bnoTemp = bno.getTemp();
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); 
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  float accMagnitude = sqrt(pow(acc.x(),2) + pow(acc.y(),2) + pow(acc.z(),2));

  loopTime = millis();
  
  // So we dont have to assign values to each element 1 by 1
  float temp[numElements] = {currentTime, state, bnoTemp, altitude, acc.x(), acc.y(), acc.z(), gyr.x(), gyr.y(), gyr.z(), mag.x(), mag.y(), mag.z(), accMagnitude, 0, 0, 0, 0};

  if(bno.getMode() == MODE_NDOF){
    imu::Quaternion quat = bno.getQuat();
    temp[numElements-4] = quat.w();
    temp[numElements-3] = quat.x();
    temp[numElements-2] = quat.y();
    temp[numElements-1] = quat.z();
  }
  memcpy(all_data, temp, sizeof(temp));
  
}

void cycle_array(float altitude, float accMagnitude){ //Might be 0s at first, but no foul
  uint8_t altStability = 1;
  uint8_t accStability = 1;
  for(int i=(runningDataSize-1); i>1; i--){
    if(altitudes[i-1] == -1){ //Only for altitudes because the first state triggers based on a comparison of values within this
      altitudes[i] = altitude;
    }else{
      altitudes[i] = altitudes[i-1];
    }
    accels[i] = accels[i-1];
    if(abs(altitudes[i]-altitude) > 3){ altStability = 0; }
    if(abs(accels[i]-accMagnitude) > 2){ accStability = 0; }
  }
  altitudes[1] = altitude;
  accels[1] = accMagnitude;
  altitudes[0] = (float)altStability;
  accels[0] = (float)accStability;
}

void save_data(){
  String dataEntry = "";
  for(int i=0; i<numElements-1; i++){
    dataEntry += (String(all_data[i]) + ",");
  }
  dataEntry += String(all_data[numElements-1]); // Last data point doesn't need a comma
  flightData.println(dataEntry);
  Serial.println(dataEntry);
  flightData.flush();
}

//                                 ==== BNO Writing, Modifying, Reading ====
// Writes data to one register on the chosen i2c_address
void i2c_write(uint8_t i2c_addr, uint8_t reg, uint8_t data){
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);  // send stop
}

// Writes to the Acceleration config register to change the max +- g's
void bno_change_range(uint8_t measurement, uint8_t range){
  uint8_t originalMode = bno.getMode();
  bno_change_mode(MODE_CONFIG);
  i2c_write(BNO_ADDR, PAGE_ID, 1);
  i2c_write(BNO_ADDR, measurement, range);    // +- 2; 0x0C, +- 4; 0x0D, +- 8; 0x0E, +- 16; 0x0F
  bno_change_mode(originalMode);
}

// BNO Library has own setMode() function, but for it to work properly we need delays and situational mode changes as well
void bno_change_mode(uint8_t mode){
  uint8_t originalMode = bno.getMode();
  if(originalMode == MODE_CONFIG || originalMode == 0){
    i2c_write(BNO_ADDR, PAGE_ID, 0);
    bno.setMode(mode);
    delay(10);
  }else if(mode == MODE_CONFIG){
    bno.setMode(MODE_CONFIG);
    delay(25);
  }else{
    bno.setMode(MODE_CONFIG);
    delay(25);
    bno.setMode(mode);
    delay(10);
  }
}
