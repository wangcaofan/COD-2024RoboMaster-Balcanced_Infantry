/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bmi088_reg.h
  * @brief          : bmi088 interfaces functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : none
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DEVICE_BMI088_REG_H
#define DEVICE_BMI088_REG_H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* Exported defines -----------------------------------------------------------*/

/*---------------------------------- accelerator register address ----------------------------------*/
#define BMI088_ACC_CHIP_ID 0x00 // the register is  " Who am I "
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
#define BMI088_FATAL_ERROR_SHFITS 0x0
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

#define BMI088_ACC_STATUS 0x03
#define BMI088_ACCEL_DRDY_SHFITS 0x7
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

#define BMI088_ACCEL_XOUT_L 0x12   /*!< Accelerator Triaxial Output Register */
#define BMI088_ACCEL_XOUT_M 0x13
#define BMI088_ACCEL_YOUT_L 0x14
#define BMI088_ACCEL_YOUT_M 0x15
#define BMI088_ACCEL_ZOUT_L 0x16
#define BMI088_ACCEL_ZOUT_M 0x17

#define BMI088_SENSORTIME_DATA_L 0x18
#define BMI088_SENSORTIME_DATA_M 0x19
#define BMI088_SENSORTIME_DATA_H 0x1A

#define BMI088_ACC_INT_STAT_1 0x1D
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

#define BMI088_TEMP_M 0x22

#define BMI088_TEMP_L 0x23

#define BMI088_ACC_CONF 0x40    /*!< accelerator config register */

/**
 * @brief bit7-bit4,sampling frequency.
 */
#define BMI088_ACC_CONF_MUST_Set 0x80
#define BMI088_ACC_BWP_SHFITS 0x4
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)    /*!< 4 times sampling frequency */
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)    /*!< 2 times sampling frequency */
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)  /*!< normal sampling frequency */

/**
 * @brief bit3-bit0,Output frequency.
 */
#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)  //12.5hz
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)    //25hz
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)    //50hz
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)   //100hz
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)   //200hz
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)   //400hz
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)   //800hz
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)  //1.6khz

#define BMI088_ACC_RANGE        0x41  /*!< accelerator scoping register */
#define BMI088_ACC_RANGE_SHFITS 0x00  /*!< number of shift */
#define BMI088_ACC_RANGE_3G     (0x0 << BMI088_ACC_RANGE_SHFITS)  //+-3g
#define BMI088_ACC_RANGE_6G     (0x1 << BMI088_ACC_RANGE_SHFITS)  //+-6g
#define BMI088_ACC_RANGE_12G    (0x2 << BMI088_ACC_RANGE_SHFITS)  //+-12g
#define BMI088_ACC_RANGE_24G    (0x3 << BMI088_ACC_RANGE_SHFITS)  //+-24g

#define BMI088_INT1_IO_CTRL 0x53    /*!< configure INT1 input and output pins */
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)  /*!< configure INT1 as output pins */
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)  /*!< push-pull output */
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)  /*!< open-drain output */
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)  /*!< pull down */
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS) /*!< pull up */

#define BMI088_INT2_IO_CTRL 0x54
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)

/**
 * @brief Interrupts are mapped to INT1 and INT2, and then can be triggered by external interrupts. 
 *        Of course, this is of course mapped to INT1, because only INT1 is configured.
 */
#define BMI088_INT_MAP_DATA 0x58  
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS)  /*!< mapped to INT2 */
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)  /*!< mapped to INT1 */

#define BMI088_ACC_SELF_TEST 0x6D
#define BMI088_ACC_SELF_TEST_OFF 0x00
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09

/**
 * @brief Switches the accelerator into suspend mode, while data collection stops.
 */
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03  /*!< pause */
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00   /*!< start */

/**
 * @brief Turn on or off the accelerator, the value below should be set.
 */
#define BMI088_ACC_PWR_CTRL 0x7D  
#define BMI088_ACC_ENABLE_ACC_OFF 0x00   /*!< Turn off the accelerator */
#define BMI088_ACC_ENABLE_ACC_ON 0x04    /*!< Turn on  the accelerator */

#define BMI088_ACC_SOFTRESET 0x7E   /*!< Software Reset Register */
#define BMI088_ACC_SOFTRESET_VALUE 0xB6  /*!< The value written by the software reset register */

/*---------------------------------- gyroscope register address ----------------------------------*/
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

#define BMI088_GYRO_XOUT_L 0x02
#define BMI088_GYRO_XOUT_H 0x03
#define BMI088_GYRO_YOUT_L 0x04
#define BMI088_GYRO_YOUT_H 0x05
#define BMI088_GYRO_ZOUT_L 0x06
#define BMI088_GYRO_ZOUT_H 0x07

#define BMI088_GYRO_INT_STAT_1 0x0A
#define BMI088_GYRO_DYDR_SHFITS 0x7
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

#define BMI088_GYRO_RANGE 0x0F  /*!< Angular rate range and resolution */
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)  /*!< +-2000°/s */
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)  /*!< +-1000°/s */
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)   /*!< +-500°/s */
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)   /*!< +-250°/s */
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)   /*!< +-125°/s */

#define BMI088_GYRO_BANDWIDTH 0x10   /*!< Set rate data filter bandwidth and output rate */

/**
 * @brief The first of the following indicates the transmission data rate, and the second indicates the bandwidth, 
 *        such as BMI088_GYRO_2000_532_HZ, the data transmission rate is 2kHZ, and the bandwidth is 532hz
 */
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80
#define BMI088_GYRO_2000_532_HZ 0x00
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02
#define BMI088_GYRO_400_47_HZ 0x03
#define BMI088_GYRO_200_23_HZ 0x04
#define BMI088_GYRO_100_12_HZ 0x05
#define BMI088_GYRO_200_64_HZ 0x06
#define BMI088_GYRO_100_32_HZ 0x07

#define BMI088_GYRO_LPM1 0x11  /*!< Power mode selection register */
#define BMI088_GYRO_NORMAL_MODE 0x00   /*!< normal mode */
#define BMI088_GYRO_SUSPEND_MODE 0x80  /*!< suspend mode */
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20  /*!< deep suspend mode */

#define BMI088_GYRO_SOFTRESET 0x14  /*!< reset sensor register */
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6  /*!< Send 0xB6 to this register to reset */

#define BMI088_GYRO_CTRL 0x15   /*!< Data Interrupt Trigger Register */
#define BMI088_DRDY_OFF 0x00    /*!< No data, ready for interrupt trigger */
#define BMI088_DRDY_ON 0x80     /*!< Allow new data to trigger a new data interrupt */

#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16  /*!< Interrupt Pin Configuration Register */
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)   /*!< INT4 push-pull output */
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)   /*!< INT4 open drain output */
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)  /*!< INT4 pull down */
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS) /*!< INT4 pull up */
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)  /*!< INT3 push-pull output */
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)  /*!< INT3 open drain output */
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)  /*!< INT3 pull down */
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS) /*!< INT3 pull up */

#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18  /*!< interrupt map register */
#define BMI088_GYRO_DRDY_IO_OFF 0x00   /*!< no mapping */
#define BMI088_GYRO_DRDY_IO_INT3 0x01  /*!< mapping to INT3 */
#define BMI088_GYRO_DRDY_IO_INT4 0x80  /*!< mapping to INT4 */
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4)   /*!< mapping to INT3 and INT4 */

#define BMI088_GYRO_SELF_TEST 0x3C
#define BMI088_GYRO_RATE_OK_SHFITS 0x4
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)

/* Exported functions prototypes ---------------------------------------------*/

#endif //DEVICE_BMI088_REG_H
