/* USER CODE BEGIN Header */
/**
  *******************************************************************************
  * @file           : bmi088.c
  * @brief          : bmi088 interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "bsp_tick.h"


/* Private define ------------------------------------------------------------*/
#if defined(BMI088_USE_SPI)

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief Write the single register value to the sensor
  */
static void BMI088_Write_Single_Reg(uint8_t, uint8_t);
/**
  * @brief Read the single register value to the sensor
  */
static void BMI088_Read_Single_Reg(uint8_t, uint8_t *);
/**
  * @brief Read the multi register value to the sensor
  */
static void BMI088_Read_Multi_Reg(uint8_t, uint8_t *, uint8_t);

/**
 * @brief macro definition of the BMI088_Accel_Write_Single_Reg that write a single data in the specified accelerator register
 * @param reg: the specified register address
 * @param data: the single data
 * @retval None
 */
#define BMI088_Accel_Write_Single_Reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    }
//------------------------------------------------------------------------------


/**
 * @brief macro definition of the BMI088_Accel_Read_Single_Reg that read a single data in the specified accelerator register
 * @param reg: the specified register address
 * @param data: the single data
 * @retval None
 */
#define BMI088_Accel_Read_Single_Reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_Read_Write_Byte((reg) | 0x80);   \
        BMI088_Read_Write_Byte(0x55);           \
        (data) = BMI088_Read_Write_Byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
//------------------------------------------------------------------------------


/**
 * @brief macro definition of the BMI088_Accel_Read_Multi_Reg that read multi datas in the specified accelerator register
 * @param reg: the specified register address
 * @param data: the multi datas
 * @param len: the length of data
 * @retval None
 */
#define BMI088_Accel_Read_Multi_Reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_Read_Write_Byte((reg) | 0x80);      \
        BMI088_Read_Multi_Reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
//------------------------------------------------------------------------------


/**
 * @brief macro definition of the BMI088_Gyro_Write_Single_Reg that write a single data in the specified gyro register
 * @param reg: the specified register address
 * @param data: the single data
 * @retval None
 */
#define BMI088_Gyro_Write_Single_Reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Write_Single_Reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
//------------------------------------------------------------------------------


/**
 * @brief macro definition of the BMI088_Gyro_Read_Single_Reg that read a single data in the specified gyro register
 * @param reg: the specified register address
 * @param data: the single data
 * @retval None
 */
#define BMI088_Gyro_Read_Single_Reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_Read_Single_Reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
//------------------------------------------------------------------------------


/**
 * @brief macro definition of the BMI088_Gyro_Read_Multi_Reg that read multi datas in the specified gyro register
 * @param reg: the specified register address
 * @param data: the multi datas
 * @param len: the length of data
 * @retval None
 */
#define BMI088_Gyro_Read_Multi_Reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_Read_Multi_Reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }
//------------------------------------------------------------------------------


#endif

/* Private variables ---------------------------------------------------------*/

/**
  * @brief 3 times sampling frequency  Accelerator 
  */
static float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;  

/**
  * @brief 2000 byte length gyro
  */
static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

/**
  * @brief structure that contains the information of BMI088
  */
BMI088_Info_Typedef BMI088_Info;

/**
  * @brief BMI088 Accelerator configuration data and Error Status
  */
static uint8_t Accel_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    /* Turn on accelerometer */
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},   

    /* Pause mode */
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR}, 

    /* Acceleration Configuration */
    {BMI088_ACC_CONF,  (BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set), BMI088_ACC_CONF_ERROR}, 

    /* Accelerometer setting range */ 
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},  

    /* INT1 Configuration input and output pin */ 
    {BMI088_INT1_IO_CTRL, (BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW), BMI088_INT1_IO_CTRL_ERROR}, 

    /* interrupt map pin */
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}  
};

/**
  * @brief BMI088 Gyro configuration data and Error Status
  */
static uint8_t Gyro_Register_ConfigurationData_ErrorStatus[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    /* Angular rate and resolution */
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR}, 

    /* Data Transfer Rate and Bandwidth Settings */
    {BMI088_GYRO_BANDWIDTH, (BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set), BMI088_GYRO_BANDWIDTH_ERROR}, 

    /* Power Mode Selection Register */
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},   

    /* Data Interrupt Trigger Register */
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},   

    /* Interrupt Pin Trigger Register */
    {BMI088_GYRO_INT3_INT4_IO_CONF, (BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW), BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},  

    /* interrupt map register */
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}   
};


/**
  * @brief Initializes the accelerator according to writing the specified data 
  *        to the internal configuration registers of the sensor.
  * @param None
  * @retval None
  */
static BMI088_Status_e BMI088_Accel_Init(void)
{
    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the accelerator ID address */
	  BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    /* read again */
    BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* accelerator software reset ------------------------------------------------*/
    /* write 0xB6 to the register ACC_SOFTRESET that address is 0x7E to reset software */
    BMI088_Accel_Write_Single_Reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE); 
    /* software reset waiting time, there is 80ms */
    Delay_ms(BMI088_LONG_DELAY_TIME);

    /* check the communication again ------------------------------------------------*/
    BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Accel_Read_Single_Reg(BMI088_ACC_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* if the value of the 0x00 register is not 0x1E,reset failed */
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* config the accelerator sensor */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
				/* Write the configuration values in the internal configuration registers of the sensor : */
				/*!< [0][0]  BMI088_ACC_PWR_CTRL 0x7D                Turn on or off the accelerator register */
			  /*!< [0][1]  BMI088_ACC_ENABLE_ACC_ON 0x04           Turn on the accelerator */
			  /*!< [1][0]  BMI088_ACC_PWR_CONF 0x7C                Switches the accelerator mode register */
			  /*!< [1][1]  BMI088_ACC_PWR_ACTIVE_MODE 0x00         start  */
			  /*!< [2][0]  BMI088_ACC_CONF 0x40                    accelerator config register */
        /*!< [2][1]  BMI088_ACC_CONF_DATA 0xAB               BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS): normal sampling frequency  */
        /*!<                                                 | BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS): 800hz output frequency */  
        /*!<                                                 | BMI088_ACC_CONF_MUST_Set 0x80 */
			  /*!< [3][0]  BMI088_ACC_RANGE 0x41                   accelerator scoping register */
			  /*!< [3][1]  BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)   +-3g */
			  /*!< [4][0]  BMI088_INT1_IO_CTRL 0x53                configure INT1 input and output pins */
			  /*!< [4][1]  BMI088_INT1_IO_CTRL_DATA 0x8            BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS): configure INT1 as output pins */ 
        /*!<                                                 | BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS): push-pull output */  
        /*!<                                                 | BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS): pull down */
			  /*!< [5][0]  BMI088_INT_MAP_DATA 0x58                interrupts mapping register */
			  /*!< [5][1]  BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)  interrupts are mapped to INT1 */
        BMI088_Accel_Write_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1]); 
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* read the configuration */
        BMI088_Accel_Read_Single_Reg(Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        
        /* check the configuration and return the specified error */
        if (res != Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][1])
        {
            return (BMI088_Status_e)Accel_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
        }
    }

    /* no error */
    return BMI088_NO_ERROR;  
}
//------------------------------------------------------------------------------


/**
  * @brief Initializes the gyro according to writing the specified data 
  *        to the internal configuration registers of the sensor.
  * @param None
  * @retval None
  */
static BMI088_Status_e BMI088_Gyro_Init(void)
{
    uint8_t res = 0;

    /* check the communication ------------------------------------------------*/
    /* read the gyro ID address */
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    /* read again */
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    /* waiting 150us */
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* gyro software reset ------------------------------------------------*/
    /* write 0xB6 to the register GYRO_SOFTRESET that address is 0x14 to reset software */
    BMI088_Gyro_Write_Single_Reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    /* software reset waiting time, there is 80ms */
    Delay_ms(BMI088_LONG_DELAY_TIME);

    /* check the communication again ------------------------------------------------*/
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_Gyro_Read_Single_Reg(BMI088_GYRO_CHIP_ID, res);
    Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    /* if the value of the 0x00 register is not 0x0F,reset failed */
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    /* config the gyro sensor */
    for (uint8_t write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
				/* Write the configuration values in the internal configuration registers of the sensor : */
				/*!< [0][0]  BMI088_GYRO_RANGE 0x0F                angular rate range and resolution register */
			  /*!< [0][1]  BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)  //+-2000°/s */
			  /*!< [1][0]  BMI088_GYRO_BANDWIDTH 0x10               set rate data filter bandwidth and output rate register */
			  /*!< [1][1]  BMI088_GYRO_2000_532_HZ                  set the data transmission rate is 2kHZ, and the bandwidth is 532hz */
			  /*!< [2][0]  BMI088_GYRO_LPM1 0x11                    power mode selection register */
			  /*!< [2][1]  BMI088_GYRO_NORMAL_MODE 0x00             normal mode */
			  /*!< [3][0]  BMI088_GYRO_CTRL 0x15                    data interrupt trigger Register */
			  /*!< [3][1]  BMI088_DRDY_ON 0x80                      allow new data to trigger a new data interrupt */
			  /*!< [4][0]  BMI088_GYRO_INT3_INT4_IO_CONF 0x16       interrupt pin configuration register */
			  /*!< [4][1]  BMI088_GYRO_INT3_INT4_IO_CONF_DATA 0x0   BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS): INT3 push-pull output  */
        /*!<                                                  | BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS): INT3 pull down  */
			  /*!< [5][0]  BMI088_GYRO_INT3_INT4_IO_MAP 0x18        interrupt map register */
			  /*!< [5][1]  BMI088_GYRO_DRDY_IO_INT3 0x01            mapping to INT3 */
        BMI088_Gyro_Write_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1]);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* read the configuration */
        BMI088_Gyro_Read_Single_Reg(Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][0], res);
        /* waiting 150us */
        Delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        /* check the configuration and return the specified error */
        if (res != Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][1])
        {
            return (BMI088_Status_e)Gyro_Register_ConfigurationData_ErrorStatus[write_reg_num][2];
        }
    }

    /* no error */
    return BMI088_NO_ERROR;
}
//------------------------------------------------------------------------------


/**
  * @brief Updates the BMI088 offsets.
  * @param BMI088_Info: pointer to a BMI088_Info_Typedef structure that
  *         contains the information  for the BMI088.
  * @retval None
  */
static void BMI088_Offset_Update(BMI088_Info_Typedef *BMI088_Info)
{
#if IMU_Calibration_ENABLE /* ENABLE the BMI088 Calibration */
	
	uint8_t buf[8] = {0,};

  for(uint16_t i = 0; i < 5000; i++)
  {
    /* read the accelerator multi data */
    BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->mpu_info.accelx = (int16_t)((buf[1]) << 8) | buf[0];
    BMI088_Info->mpu_info.accely = (int16_t)((buf[3]) << 8) | buf[2];
    BMI088_Info->mpu_info.accelz = (int16_t)((buf[5]) << 8) | buf[4];

    /* read the gyro multi data */
    BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)   
    {
      BMI088_Info->mpu_info.gyrox = (int16_t)((buf[3]) << 8) | buf[2];
      BMI088_Info->mpu_info.gyroy = (int16_t)((buf[5]) << 8) | buf[4];
      BMI088_Info->mpu_info.gyroz = (int16_t)((buf[7]) << 8) | buf[6];

      /* update the gyro offsets */
      BMI088_Info->offsets_gyrox += BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyrox;
      BMI088_Info->offsets_gyroy += BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroy;
      BMI088_Info->offsets_gyroz += BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroz;
    }
    /* waiting 1ms */
    Delay_ms(1);
  }

  BMI088_Info->offsets_gyrox = BMI088_Info->offsets_gyrox / 5000.f;
  BMI088_Info->offsets_gyroy = BMI088_Info->offsets_gyroy / 5000.f; 
  BMI088_Info->offsets_gyroz = BMI088_Info->offsets_gyroz / 5000.f;
	
#else /* DISABLE the BMI088 Calibration */
	/* store the previous offsets */
  BMI088_Info->offsets_gyrox = 0.0101350538f;
  BMI088_Info->offsets_gyroy = -0.0067467244f;
  BMI088_Info->offsets_gyroz = 0.00014622093f;
#endif

  /* set the offset init flag */
  BMI088_Info->offsets_init = true;
}
//------------------------------------------------------------------------------


/**
 * 
  * @brief Initializes the BMI088 according to writing the specified data 
  *        to the internal configuration registers of the sensor.
  * @param None
  * @retval Status
  */
void BMI088_Init(void)
{
  BMI088_Status_e status = BMI088_NO_ERROR;

  /* Initializes the BMI088 */
  do
  {
    /* Judge the accelerator configuration */
    status |= BMI088_Accel_Init();
    /* Judge the gyro configuration */
    status |= BMI088_Gyro_Init();
    /* waiting 2ms */
    Delay_ms(2);
  }while(status);

  /* update the bmi088 offset */
  BMI088_Offset_Update(&BMI088_Info);
}
//------------------------------------------------------------------------------


/**
  * @brief Updates the BMI088 Information.
  * @param BMI088_Info: pointer to a BMI088_Info_Typedef structure that
  *         contains the information  for the BMI088.
  * @retval None
  */
void BMI088_Info_Update(BMI088_Info_Typedef *BMI088_Info)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};

    /* read the accelerator multi data */
		BMI088_Accel_Read_Multi_Reg(BMI088_ACCEL_XOUT_L, buf, 6);
    BMI088_Info->mpu_info.accelx = (int16_t)((buf[1] << 8) | buf[0]);
    BMI088_Info->mpu_info.accely = (int16_t)((buf[3] << 8) | buf[2]);
    BMI088_Info->mpu_info.accelz = (int16_t)((buf[5] << 8) | buf[4]);

    /* converts the accelerator data */
    BMI088_Info->accel[0] = BMI088_ACCEL_SEN * BMI088_Info->mpu_info.accelx;
		BMI088_Info->accel[1] = BMI088_ACCEL_SEN * BMI088_Info->mpu_info.accely;
		BMI088_Info->accel[2] = BMI088_ACCEL_SEN * BMI088_Info->mpu_info.accelz;

    /* read the temperature */
    BMI088_Accel_Read_Multi_Reg(BMI088_TEMP_M, buf, 2);
    BMI088_Info->mpu_info.temperature = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (BMI088_Info->mpu_info.temperature > 1023) BMI088_Info->mpu_info.temperature -= 2048;

    /* converts the temperature data */
    BMI088_Info->temperature = BMI088_Info->mpu_info.temperature * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    /* read the gyro multi data */
		BMI088_Gyro_Read_Multi_Reg(BMI088_GYRO_CHIP_ID, buf, 8);
    /* check the ID */
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)   
    {
      BMI088_Info->mpu_info.gyrox = (int16_t)((buf[3] << 8) | buf[2]);
      BMI088_Info->mpu_info.gyroy = (int16_t)((buf[5] << 8) | buf[4]);
      BMI088_Info->mpu_info.gyroz = (int16_t)((buf[7] << 8) | buf[6]);
    }

    /* converts the gyro data */
		BMI088_Info->gyro[0] = BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyrox - BMI088_Info->offsets_gyrox;
	  BMI088_Info->gyro[1] = BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroy - BMI088_Info->offsets_gyroy;
	  BMI088_Info->gyro[2] = BMI088_GYRO_SEN * BMI088_Info->mpu_info.gyroz - BMI088_Info->offsets_gyroz;
}
//------------------------------------------------------------------------------


#if defined(BMI088_USE_SPI)
/**
  * @brief Write the single register value to the sensor
  * @param reg: the specified register address
  * @param data: the specified register value
  * @retval none
  */
static void BMI088_Write_Single_Reg(uint8_t reg, uint8_t data)
{
    /* write the address to the sensor */
    BMI088_Read_Write_Byte(reg);  

    /* write the register value to the sensor */
    BMI088_Read_Write_Byte(data);
}
//------------------------------------------------------------------------------


/**
  * @brief Read the single register value to the sensor
  * @param reg: the specified register address
  * @param return_data: pointer to the specified register value
  * @retval none
  */
static void BMI088_Read_Single_Reg(uint8_t reg, uint8_t *return_data)
{
    /**
     * @brief As mentioned in the manual, in the gyroscope mode, 
     *        write 0x80 to the register to trigger a new data interrupt, 
     *        and the other modes are the same. 
     */
    BMI088_Read_Write_Byte(reg | 0x80); 

    /* write/read the register value to the sensor */
    *return_data = BMI088_Read_Write_Byte(0x55);
}
//------------------------------------------------------------------------------


/**
  * @brief Read the multi register value to the sensor
  * @param reg: the specified register address
  * @param buf: pointer to the specified register value
  * @param len: the length of specified register value
  * @retval none
  */
static void BMI088_Read_Multi_Reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* trigger a new data interrupt */
    BMI088_Read_Write_Byte(reg | 0x80);

    while (len != 0)
    {
        /* write/read the register value to the sensor */
        *buf = BMI088_Read_Write_Byte(0x55); 
        buf++;
        len--;
    }
}
//------------------------------------------------------------------------------


#endif

