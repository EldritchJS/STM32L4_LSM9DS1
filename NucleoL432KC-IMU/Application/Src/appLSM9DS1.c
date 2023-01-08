#include "cmsis_os.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "appI2C.h"
#include "appUSART.h"
#include "lsm9ds1_reg.h"
#include "appLSM9DS1.h"
#include <string.h>
#include <stdio.h>
 
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

#if 1
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
static int32_t stm32l4_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t stm32l4_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len);
static int32_t stm32l4_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len);
static int32_t stm32l4_read_mag(void *handle, uint8_t reg,
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
  dev_ctx_imu.write_reg = stm32l4_write_imu;
  dev_ctx_imu.read_reg = stm32l4_read_imu;
  dev_ctx_imu.handle = (void *)&imu_bus;
  /* Initialize magnetic sensors driver interface */
  dev_ctx_mag.write_reg = stm32l4_write_mag;
  dev_ctx_mag.read_reg = stm32l4_read_mag;
  dev_ctx_mag.handle = (void *)&mag_bus;

  osDelay(BOOT_TIME);

  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);

  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID)
  {
	while (1)
	{
	  osDelay(1);
	}
  }
  else
  {
    USART1TxStr("whoami passed\r\n");
  }

  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu,
                                PROPERTY_ENABLE);
  /* Set full scale */
   lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
   lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
   lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);
   lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
   lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu,
                                      LSM9DS1_LP_ODR_DIV_50);
   lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
   /* Gyroscope filtering chain */
   lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu,
                                      LSM9DS1_LP_ULTRA_LIGHT);
   lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
   lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu,
                                  LSM9DS1_LPF1_HPF_LPF2_OUT);
   /* Set Output Data Rate / Power mode */
   lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
   lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

  /* Infinite loop */
  for(;;)
  {
    /* Read device status register */
	lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

	if ( reg.status_imu.xlda && reg.status_imu.gda )
	{
	  /* Read imu data */
	  memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	  memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
	  lsm9ds1_acceleration_raw_get(&dev_ctx_imu,
	                               data_raw_acceleration);
	  lsm9ds1_angular_rate_raw_get(&dev_ctx_imu,
	                               data_raw_angular_rate);
	  acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(
	                         data_raw_acceleration[0]);
	  acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(
	                         data_raw_acceleration[1]);
	  acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(
	                         data_raw_acceleration[2]);
	  angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(
	                           data_raw_angular_rate[0]);
	  angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(
	                           data_raw_angular_rate[1]);
	  angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(
	                           data_raw_angular_rate[2]);
	  sprintf((char *)tx_buffer,
	          "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
	          acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
	          angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
	  USART1TxStr((char *)tx_buffer);
	}

	if ( reg.status_mag.zyxda )
	{
	  /* Read magnetometer data */
	  memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
	  lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field);
	  magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
	                               data_raw_magnetic_field[0]);
	  magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
	                               data_raw_magnetic_field[1]);
	  magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
	                               data_raw_magnetic_field[2]);
	  sprintf((char *)tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
	          magnetic_field_mgauss[0], magnetic_field_mgauss[1],
	          magnetic_field_mgauss[2]);
	  USART1TxStr((char *)tx_buffer);
	}
    osDelay(500);
  }
}

static int32_t stm32l4_write_imu(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  I2C1Tx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);

  //HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
    //                I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t stm32l4_read_imu(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  I2C1Rx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);
  //HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
    //               I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

static int32_t stm32l4_write_mag(void *handle, uint8_t reg,
                                  const uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;

  reg |= 0x80;
  I2C1Tx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);

  //HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
    //                I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
  return 0;
}

static int32_t stm32l4_read_mag(void *handle, uint8_t reg,
                                 uint8_t *bufp, uint16_t len)
{
  sensbus_t *sensbus = (sensbus_t *)handle;
  reg |= 0x80;
  I2C1Rx(sensbus->i2c_address, reg, (uint8_t*) bufp, len);
  //HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
    //               I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
  return 0;
}

