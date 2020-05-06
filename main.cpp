#include <mbed.h>
#include <iostream>
#include <MPU6050.h>


//----------------------SAMD SETUP----------------------------------------
#define REG_PORT_DIR0 
#define REG_PORT_DIRCLR0
#define REG_PORT_DIRSET0
#define REG_PORT_DIRTGL0
#define REG_PORT_OUT0
#define REG_PORT_OUTCLR0
#define REG_PORT_OUTGL0
#define REG_PORT_IN0              
#define REG_PORT_CTRL0
#define REG_PORT_WRCONOFIG0
#define REG_PORT_PMUX0
#define REG_PORT_PINCFG0

 // on board switch, configure to act as on off switch for the starting of recording from accelerometer [page 14]
#define SW0 PORT_PA15 

#define LED0 PORT_PB30
//----------------------MPU SETUP----------------------------------------

// Address used to access data
#define MPU_DEFAULT_I2C_ADDR 0x68

// Interrupt enable register
#define MPU_INT_ENABLE 0x38 // Register as follows: {-, -, -, FIFO_OFLOW_EN, I2C_MST_INT_EN, -, -, DATA_RDY_EN}

// Interrupt status register
#define MPU_INT_STATUS 0x3A // Register as follows: {-, -, -, FIFO_OFLOW_INT, I2C_MST_INT, -, -, DATA_RDY_INT}

// MPU configuration register
#define MPU_CONFIG 0x1A // Register as follows: {-, -, EXT_SYNC_SET[3 bits], DLPF_CONFIG[3 bits]}




//--------------------------------FSYNC ISRS----------------------------------
// EXT_SYCN_SET parameters to set the FSYNC bit location
// #define MPU_CONFIG_FSYNC_DISABLE   0
// #define MPU_CONFIG_FSYNC_TEMP_OUT  1
// #define MPU_CONFIG_FSYNC_GYRO_XOUT 2
// #define MPU_CONFIG_FSYNC_GYRO_YOUT 3
// #define MPU_CONFIG_FSYNC_GYRO_ZOUT 4
// #define MPU_CONFIG_FSYNC_ACC_XOUT  5
// #define MPU_CONFIG_FSYNC_ACC_YOUT  6
// #define MPU_CONFIG_FSYNC_ACC_ZOUT  7

//-----------------------Digital Low Pass Filters 
// DLPF_CONFIG parameters - for more information see the datasheet
                            // |        Accelerometer        |                Gyroscope               |
							// |  Bandwidth/Hz  |  Delay/ms  |  Bandwidth/Hz  |  Delay/ms  |  Fs/kHz  |
/* #define MPU_CONFIG_DLPF_0 0 // |      260       |     0.0    |      256       |     0.98   |     8    |
#define MPU_CONFIG_DLPF_1 1 // |      184       |     2.0    |      188       |     1.9    |     1    |
#define MPU_CONFIG_DLPF_2 2 // |       94       |     3.0    |       98       |     2.8    |     1    |
#define MPU_CONFIG_DLPF_3 3 // |       44       |     4.9    |       42       |     4.8    |     1    |
#define MPU_CONFIG_DLPF_4 4 // |       21       |     8.5    |       20       |     8.3    |     1    |
#define MPU_CONFIG_DLPF_5 5 // |       10       |    13.8    |       10       |    13.4    |     1    |
#define MPU_CONFIG_DLPF_6 6 // |        5       |    19.0    |        5       |    18.6    |     1    |
#define MPU_CONFIG_DLPF_7 7 / / |           RESERVED          |           RESERVED          |     8    |*/
// ---------------------------------------------

// ------------ Gyroscope Parameters -----------
// Gyro configuration register
#define MPU_GYRO_CONFIG 0x1B // Register as follows: {XG_ST, YG_ST, ZG_ST, FS_SEL[2 bits], -, -, -}

// FS_SEL parameters - Gyro sensitivity parameters
//#define MPU_GYRO_SENS_250  0 // ± 250 °/s
//#define MPU_GYRO_SENS_500  1 // ± 500 °/s
#define MPU_GYRO_SENS_1000 2 // ± 1000 °/s
//#define MPU_GYRO_SENS_2000 3 // ± 2000 °/s

// Gyro scaling parameters - based on sensitivity above
//#define MPU_GYRO_SCALE_250  131  // for ± 250 °/s
//#define MPU_GYRO_SCALE_500  65.5 // for ± 500 °/s
#define MPU_GYRO_SCALE_1000 32.8 // for ± 1000 °/s
//#define MPU_GYRO_SCALE_2000 16.4 // for ± 2000 °/s

// Gyro reading registers - Each register is 8 bits
#define MPU_GYRO_X1 0x43 // Most significant byte
#define MPU_GYRO_X2 0x44 // Least significant byte
#define MPU_GYRO_Y1 0x45
#define MPU_GYRO_Y2 0x46
#define MPU_GYRO_Z1 0x47
#define MPU_GYRO_Z2 0x48
// ---------------------------------------------

// ---------- Accelerometer Parameters ---------
// Accelerometer configuration register
#define MPU_ACC_CONFIG 0x1C // Register as follows: {XA_ST, YA_ST, ZA_ST, AFS_SEL[2 bits], -, -, -}

// AFS_SEL parameters - Accelerometer sensitivity parameters
#define MPU_ACC_SENS_2  0 // ±2g
//#define MPU_ACC_SENS_4  1 // ±4g
//#define MPU_ACC_SENS_8  2 // ±8g
//#define MPU_ACC_SENS_16 3 // ±16g

// Accelerometer scaling parameters - based on sensitivity above
#define MPU_ACC_SCALE_2  16384 // for ±2g
//#define MPU_ACC_SCALE_4  8192  // for ±4g
//#define MPU_ACC_SCALE_8  4096  // for ±8g
//#define MPU_ACC_SCALE_16 2048  // for ±16g

// Accelerometer reading registers - Each register is 8 bits
#define MPU_ACC_X1 0x3B // Most significant byte
#define MPU_ACC_X2 0x3C // Least significant byte
#define MPU_ACC_Y1 0x3D
#define MPU_ACC_Y2 0x3E
#define MPU_ACC_Z1 0x3F
#define MPU_ACC_Z2 0x40


// -------------- Power Registers --------------
// Power management registers
#define MPU_PWR_MGMT_1 0x6B // Register is as follows: {DEVICE_REST, SLEEP, CYCLE, -, TEMP_DISABLE, CLK_SEL[3 bits]}
#define MPU_PWR_MGMT_2 0x6C // Register is as follows: {LP_WAKE_CTRL[2 bits], STBY_XA, STBY_YA, STBY_ZA, STBY_XG, STBY_YG, STBY_ZG}

// Clock configuration values
#define MPU_PWR_MGMT_CLK_INTERNAL_8MHZ 0
#define MPU_PWR_MGMT_CLK_PLL_X_GYRO    1
#define MPU_PWR_MGMT_CLK_PLL_Y_GYRO    2
#define MPU_PWR_MGMT_CLK_PLL_Z_GYRO    3
#define MPU_PWR_MGMT_CLK_EXT_32_768KHZ 4
#define MPU_PWR_MGMT_CLK_EXT_19_2MHZ   5
#define MPU_PWR_MGMT_CLK_STOP          7

// Wakeup data frequency configuration values
//#define MPU_PWR_MGMT_WAKE_1_25HZ 0
#define MPU_PWR_MGMT_WAKE_5HZ    1
//#define MPU_PWR_MGMT_WAKE_20HZ   2
//#define MPU_PWR_MGMT_WAKE_40HZ   3

// ---------------------------------------------


// Declare a class to process and store the data
class MPU6050{
public:
	// ---------- Special class members -----------
    MPU6050();                                         // Default constructor - used for making arrays of the object
	MPU6050(bool isPiRev0);                            // Default constructor with compatibility for rev0 Pis
	MPU6050(int deviceAddress, bool isPiRev0 = false); // Constructor with additional parameters to set the address and if the Pi is rev0
	// Constructor to allow customization of basic configuration parameters
	MPU6050(int pwrMgmtMode, int gyroConfig, int accelConfig, int deviceAddress = MPU_DEFAULT_I2C_ADDR, bool isPiRev0 = false);
	MPU6050(const MPU6050& M);                         // Copy constructor
	~MPU6050();                                        // Destructor

	// ----------- Operator overloading -----------
	MPU6050& operator=(const MPU6050& M); // Assignment operator

	// ---------- Data Access Functions -----------
	void updateData();
	float getGyroX();
	float getGyroY();
	float getGyroZ();
	float getAccelX();
	float getAccelY();
	float getAccelZ();
	float getTemp();
	// --------------------------------------------

	// ----------- Data Output Function -----------
	friend std::ostream& operator<<(std::ostream& out, MPU6050& M);
	// --------------------------------------------

private:
	// I2C address of the device
	int address;

	// I2C Variables
	char fileName[11];
	int i2cHandle;

	// Functions to initialise the MPU6050 - should only be called once
	void defaultInitialise();
	void initialise(int pwrMgmtMode, int gyroConfig, int accelConfig);

	// Function to read an entire 16-bit register from the MPU6050
	//int16_t read16BitRegister(__u8 MSBRegister, __u8 LSBRegister, bool &readError);

	// Gyroscope values
	float gyroScale;
	float gyroX;
	float gyroY;
	float gyroZ;

	// Accelerometer values
	float accelScale;
	float accelX;
	float accelY;
	float accelZ;

};

int main() {
// put your setup code here, to run once:

    // Initialize the SAM system
      SystemInit(); 

    // Configure SW0 as input (pressing the button drives line to ground)    
      PORT ->Group[0].PINCFG[15].reg = PORT_PINCFG_INEN || PORT_PINCFG_PULLEN;
    // Pull-up Resistor   
      REG_PORT_OUTSET0 = SW0; 
	//
	REG_PORT_DIRSET1 = LED1; 

  
  while(1) {
  // put your main code here, to run repeatedly:
	
    // If button is pressed 
    if ((REG_PORT_IN0 & SW0) != 0){
	  // Turning on LED for troubleshooting  
	  REG_PORT_OUTSET1 = LED0;
      // Configure to start recording info from accelerometer 

    }

    else{
      //Configure to turn off reading

	  // Turning off LED for troubleshooting 
	  REG_PORT_OUTCLR1 = LED0; 
    }



    //-------------------- Example of creation/use of MPU class------------------
    // MPU6050 IMU;      // Create the MPU6050 object - this line must be changed on revision 0 Pis, or for MPU6050 modules not on address 0x68
    // IMU.updateData(); // Get data from the IMU
    // cout << IMU;      // Display all of the data about this object
    // IMU.updateData(); // Get data from the IMU to refresh it
    // cout << IMU;      // Display the refreshed data

    // Example using constructor with more configurable parameters
    // MPU6050 customIMU(MPU_PWR_MGMT_CLK_INTERNAL_8MHZ, MPU_GYRO_SENS_1000, MPU_ACC_SENS_2, MPU_DEFAULT_I2C_ADDR, false);
    // customIMU.updateData(); // Update the new IMU
    // cout << customIMU;      // Print the data
    // customIMU.updateData(); // Update the new IMU
    // cout << customIMU;      // Print the data

    return;
  }
}