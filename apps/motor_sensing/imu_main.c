/***************************************************************************
 * apps/motor_sensing/imu_main.c
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <nuttx/sensors/sensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_LOW_MASK    0xFF00
#define REG_HIGH_MASK   0x00FF
#define MPU6050_AFS_SEL 4096.0f   /* Accel scale factor */
#define SAMPLE_RATE_MS  100       /* 10 Hz sample rate */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mpu6050_imu_msg
{
  int16_t acc_x;
  int16_t acc_y;
  int16_t acc_z;
  int16_t temp;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void read_mpu6050(int fd, struct sensor_accel *acc_data)
{
  struct mpu6050_imu_msg raw_imu;
  int16_t raw_data[7];
  memset(&raw_imu, 0, sizeof(raw_imu));

  int ret = read(fd, &raw_data, sizeof(raw_data));
  if (ret != sizeof(raw_data))
    {
      printf("Failed to read IMU data\n");
      acc_data->x = 0;
      acc_data->y = 0;
      acc_data->z = 0;
      return;
    }

  /* Convert raw data */
  raw_imu.acc_x = ((raw_data[0] & REG_HIGH_MASK) << 8) +
                   ((raw_data[0] & REG_LOW_MASK) >> 8);
  raw_imu.acc_y = ((raw_data[1] & REG_HIGH_MASK) << 8) +
                   ((raw_data[1] & REG_LOW_MASK) >> 8);
  raw_imu.acc_z = ((raw_data[2] & REG_HIGH_MASK) << 8) +
                   ((raw_data[2] & REG_LOW_MASK) >> 8);

  /* Convert to g force */
  acc_data->x = raw_imu.acc_x / MPU6050_AFS_SEL;
  acc_data->y = raw_imu.acc_y / MPU6050_AFS_SEL;
  acc_data->z = raw_imu.acc_z / MPU6050_AFS_SEL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  struct sensor_accel acc_data;

  printf("MPU60x0 Accelerometer Test\n");
  printf("Sample Rate: %d ms\n", SAMPLE_RATE_MS);

  fd = open("/dev/imu0", O_RDONLY);
  if (fd < 0)
    {
      printf("Failed to open imu0\n");
      return EXIT_FAILURE;
    }

  while (1)
    {
      read_mpu6050(fd, &acc_data);

      printf("Accel (g): X=%.2f Y=%.2f Z=%.2f\n",
             acc_data.x, acc_data.y, acc_data.z);

      usleep(SAMPLE_RATE_MS * 1000);
    }

  /* Never reaches here */
  close(fd);
  return EXIT_SUCCESS;
}
