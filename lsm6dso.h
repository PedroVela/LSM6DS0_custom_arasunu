#define I2C_PORT i2c0
const uint LED_PIN_IMU = 14;    //IMU LED state
uint8_t WHO_AM_I_IMU = 0x0F;    //IMU WHO AM I
uint8_t INT1_CTRL = 0x0D;
uint8_t CTRL1_XL = 0x10;
uint8_t CTRL2_G = 0x11;
uint8_t CTRL3_C = 0x12;
uint8_t CTRL4_C = 0x13;
uint8_t CTRL5_C = 0x14;
uint8_t CTRL6_C = 0x15;
uint8_t CTRL7_G = 0x16;
uint8_t CTRL8_XL = 0x17;
uint8_t CTRL9_XL = 0x18;
uint8_t CTRL10_C = 0x19;
uint8_t STATUS_REG = 0x1E;


static int addr_IMU = 0x6B; //LSM6DSO Device Address ID

static inline int8_t sgn(float val) {//Retorna un "static inline" (Ver pág.__)
    if (val < 0) return -1;
    if (val == 0) return 0;
    return 1;
}

void AccelCalib(int N){
    int16_t rawAx, rawAy, rawAz;
    float sumX = 0.0, sumY = 0.0, sumZ = 0.0;
    uint8_t bufferAccel[2];
    uint8_t val = 0x28; // OUTX_L_A (28h) and OUTX_H_A (29h)
    for (int i = 0; i < N; i++)
    {
        sleep_ms(5);
        i2c_write_blocking(I2C_PORT, addr_IMU, &val, 1, true);
        i2c_read_blocking(I2C_PORT, addr_IMU, bufferAccel, 2, false);
        rawAx = rawData(bufferAccel);
        sumX += (float) rawAx;

        val = 0x2A; //OUTY_L_A (2Ah) and OUTY_H_A (2Bh)
        i2c_write_blocking(I2C_PORT, addr_IMU, &val, 1, true);
        i2c_read_blocking(I2C_PORT, addr_IMU, bufferAccel, 2, false);
        rawAy = rawData(bufferAccel);
        sumY += (float) rawAy;

        val = 0x2C; //OUTZ_L_A (2Ch) and OUTZ_H_A (2Dh)
        i2c_write_blocking(I2C_PORT, addr_IMU, &val, 1, true);
        i2c_read_blocking(I2C_PORT, addr_IMU, bufferAccel, 2, false);
        rawAz = rawData(bufferAccel);
        sumZ += (float) rawAz;
    }
    sumX /= N;
    sumY /= N;
    sumZ /= N;
    sumX /= 2000000.0;
    sumY /= 2000000.0;
    sumZ /= 2000000.0;
    int8_t X_OFFSET = (int8_t) sumX;
    int8_t Y_OFFSET = (int8_t) sumY;
    int8_t Z_OFFSET = (int8_t) sumZ;
}

float GyroConvertData(uint8_t buff_g[2], uint8_t mode_g){
    int16_t buff_g2 = rawData(buff_g);
    float f_gyro;
    switch (mode_g)
    {
        case 0xA2:
            f_gyro = buff_g2 / (32768.00/125.00);
            break;
        case 0x50:
            f_gyro = buff_g2 / (32768.00/250.00);
            break;
        case 0xA4:
            f_gyro = buff_g2 / (32768.00/500.00);
            break;
        case 0xA8:
            f_gyro = buff_g2 / (32768.00/1000.00);
            break;
        case 0xAC:
            f_gyro = buff_g2 / (32768.00/2000.00);
            break;
        default:
            break;
    }
    return f_gyro;
}

float AccelConvertData(uint8_t buff_a[2], uint8_t mode_a){
    int16_t buff_a2 = -rawData(buff_a);
    float f_accel;
    switch (mode_a)
    {
        case 0x76:
            f_accel = buff_a2 / (32768.00/16.00);   //realizamos el ultimo paso con el factor de escala (16G)
            break;
        case 0xAE:
            f_accel = buff_a2 / (32768.00/8.00);   //realizamos el ultimo paso con el factor de escala (8G)
            break;
        case 0xAA:
            f_accel = buff_a2 / (32768.00/4.00);   //realizamos el ultimo paso con el factor de escala (4G)
            break;
        case 0xA2:
            f_accel = buff_a2 / (32768.00/2.00);   //realizamos el ultimo paso con el factor de escala (2G)
            break;
        default:
            break;
    }
    return f_accel;
}

void IMU_init(uint8_t ACCEL_CTRL_REG, uint8_t GYRO_CTRL_REG){
    // Check to see if connnection is correct
    sleep_ms(5000);     //add a short delay to help boot up
    uint8_t reg = 0x0F;         //in the datasheet of the LSM6DSO it's in parenthesis WHO_AM_I_REG (0fh)
    uint8_t chipID[1];
    i2c_write_blocking(I2C_PORT, addr_IMU, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, addr_IMU, chipID, 1, false);
    printf("\nCHIP ID IMU = %d", chipID[0]);
    while (chipID[0] != 0x6C)  //this is the fixed value to read, the datasheet says 6Ch which means 0x6C
    {
        printf(" IMU chip ID is not correct, check connection!\n");
        gpio_put(LED_PIN_IMU, 1);
        sleep_ms(250);
        gpio_put(LED_PIN_IMU, 0);
        sleep_ms(250);
        i2c_write_blocking(I2C_PORT, addr_IMU, &WHO_AM_I_IMU, 1, true);
        i2c_read_blocking(I2C_PORT, addr_IMU, chipID, 1, false);
    }
    printf("\tIMU is succesfully connected!\n");

    for (size_t i = 0; i < 3; i++)  //3 fast blinks = connection succesful
    {
        gpio_put(LED_PIN_IMU, 1);
        sleep_ms(125);
        gpio_put(LED_PIN_IMU, 0);
        sleep_ms(125);
    }
    
   /* Startup sequence
    Once the device is powered up, it automatically downloads the calibration coefficients from the embedded flash to
    the internal registers. When the boot procedure is completed, i.e. after approximately 10 milliseconds, the
    accelerometer and gyroscope automatically enter Power-Down mode.
    To turn on the accelerometer and gather acceleration data through the primary I²C / MIPI I3CSM / SPI interface, it
    is necessary to select one of the operating modes through the CTRL1_XL register.
    The following general-purpose sequence can be used to configure the accelerometer:

    1. Write INT1_CTRL = 01h // Acc data-ready interrupt on INT1
    2. Write CTRL1_XL = 60h // Acc = 417 Hz (High-Performance mode)*/

    uint8_t data[2];    //data to be sent

//###------------------------Accelerometer activation----------------------###
    data[0] = INT1_CTRL;        //Register address
    data[1] = 0x01;              //Acc data-ready interrupt on INT1
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

    data[0] = CTRL1_XL;        //Register address
    data[1] = ACCEL_CTRL_REG; 
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);  
    /*To turn on the gyroscope and gather angular rate data through the primary I²C / MIPI I3CSM / SPI interface, it is
    necessary to select one of the operating modes through CTRL2_G.
    The following general-purpose sequence can be used to configure the gyroscope:
    1. Write INT1_CTRL = 02h // Gyro data-ready interrupt on INT1
    2. Write CTRL2_G = 60h // Gyro = 417 Hz (High-Performance mode)*/
//###-------------------------Gyroscope activation-------------------------###
    data[0] = INT1_CTRL;        //Register address
    data[1] = 0x02;              //Gyro data-ready interrupt on INT1
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

    data[0] = CTRL2_G;        //Register address
    data[1] = GYRO_CTRL_REG;          
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

//###----------------------------Settings---------------------------------###
    data[0] = CTRL3_C; //BDU address for data synchronization
    data[1] = 0x44; //BDU = 1;
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

//  Zero
    data[0] = CTRL4_C;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

//  Circular burst-mode (rounding) read from the output registers
    data[0] = CTRL5_C;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

//  High-performance operating mode enable/disable for accelerometer. 0x18 disable 0x08 enable
    data[0] = CTRL6_C;
    data[1] = 0x08;
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

//  Enables/disables high-performance operating mode for gyroscope. 0x82 disable 0x02 enable
    data[0] = CTRL7_G;
    data[1] = 0x72;
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

//  Accelerometer LPF2 and HP filter configuration. ODR/4
    data[0] = CTRL8_XL;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

//  Zero
    data[0] = CTRL9_XL;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);

//  Timestamp zero
    data[0] = CTRL10_C;
    data[1] = 0x00;
    i2c_write_blocking(I2C_PORT, addr_IMU, data, 2, true);
}

