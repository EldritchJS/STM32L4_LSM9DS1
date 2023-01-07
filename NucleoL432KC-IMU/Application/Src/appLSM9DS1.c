#include "cmsis_os.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "appI2C.h"
#include "appUSART.h"
#include "lsm9ds1_reg.h"
#include "appLSM9DS1.h"
#include <string.h>
 
#define SENSOR_BUS hi2c1
#define BOOT_TIME  20

typedef struct {
  void   *hbus;
  uint8_t i2c_address;
} sensbus_t;

static sensbus_t mag_bus = {&SENSOR_BUS,
                            LSM9DS1_MAG_I2C_ADD_H,
                           };
static sensbus_t imu_bus = {&SENSOR_BUS,
                            LSM9DS1_IMU_I2C_ADD_H,
                           };

#if 0
static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static int16_t data_raw_magnetic_field[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static uint8_t rst;
static uint8_t tx_buffer[1000];
static lsm9ds1_status_t reg;
#endif

static lsm9ds1_id_t whoamI;

osThreadId imuTaskHandle;
uint32_t imuTaskBuffer[ 128 ];
osStaticThreadDef_t imuTaskControlBlock;

void StartIMUTask(void const * argument);
static int32_t stm32l4_write_i2c(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t stm32l4_read_i2c(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);

void IMUInit(void)
{
  osThreadStaticDef(imuTask, StartIMUTask, osPriorityNormal, 0, 128, imuTaskBuffer, &imuTaskControlBlock);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
}

void StartIMUTask(void const * argument)
{
  stmdev_ctx_t dev_ctx_imu;
  stmdev_ctx_t dev_ctx_mag;

  /* Initialize inertial sensors (IMU) driver interface */
  dev_ctx_imu.write_reg = stm32l4_write_i2c;
  dev_ctx_imu.read_reg = stm32l4_read_i2c;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = stm32l4_write_i2c;
  dev_ctx_mag.read_reg = stm32l4_read_i2c;
  dev_ctx_mag.handle = (void *)&mag_bus;

  osDelay(BOOT_TIME);

  /* Infinite loop */
  for(;;)
  {
    /* Check device ID */
	lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

	if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID)
	{
	  while (1)
	  {
	    osDelay(1);
	    /* manage here device not found */
	  }
	}
	else
	{
	  USART1TxStr("whoami passed\r\n");
	}
    osDelay(5000);
  }
}

static int32_t stm32l4_write_i2c(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  reg |= 0x80;
  I2C1Tx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);

  //HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
    //                I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t stm32l4_read_i2c(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  I2C1Rx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);
  //HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
    //               I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

