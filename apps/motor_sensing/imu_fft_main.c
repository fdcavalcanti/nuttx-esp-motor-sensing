/***************************************************************************
 * apps/motor_sensing/imu_fft_main.c
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
#include <stdlib.h>
#include <math.h>
#include <nuttx/sensors/sensor.h>

#include "kiss_fft.h"
#include "tools/kiss_fftr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_LOW_MASK    0xFF00
#define REG_HIGH_MASK   0x00FF
#define MPU6050_AFS_SEL 4096.0f   /* Accel scale factor */
#define SAMPLE_RATE_MS  20        /* 50 Hz sample rate */
#define NUM_SAMPLES     128       /* FFT size */
#define SAMPLE_FREQ     50        /* Sampling frequency in Hz */

/* KissFFT configuration */
#define KISS_FFT_ALIGN_SIZE (sizeof(void*))
#define KISS_FFT_CFG_SIZE   (sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx)*(NUM_SAMPLES-1))

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
 * Private Data
 ****************************************************************************/

/* Static allocation for FFT buffers */
static kiss_fft_cpx g_fft_in[NUM_SAMPLES];
static kiss_fft_cpx g_fft_out[NUM_SAMPLES];
static char g_fft_cfg_buffer[KISS_FFT_CFG_SIZE] __attribute__((aligned(KISS_FFT_ALIGN_SIZE)));
static kiss_fft_cfg g_fft_cfg;

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

static void process_fft(float *samples, int nsamples)
{
  float freq_step;
  int i;

  /* Initialize FFT configuration if not done */
  if (g_fft_cfg == NULL)
    {
      g_fft_cfg = kiss_fft_alloc(nsamples, 0, g_fft_cfg_buffer, &g_fft_cfg_buffer);
    }

  /* Clear FFT buffers */
  memset(g_fft_in, 0, sizeof(g_fft_in));
  memset(g_fft_out, 0, sizeof(g_fft_out));

  /* Copy samples to input buffer and set imaginary part to 0 */
  for (i = 0; i < nsamples; i++)
    {
      g_fft_in[i].r = samples[i];
      g_fft_in[i].i = 0;
    }

  /* Perform FFT */
  kiss_fft(g_fft_cfg, g_fft_in, g_fft_out);

  /* Calculate frequency step */
  freq_step = (float)SAMPLE_FREQ / nsamples;

  /* Print real part of FFT output */
  printf("FFT Output (Real Part):\n");
  printf("Freq(Hz) | Magnitude\n");
  for (i = 0; i < nsamples/2 + 1; i++)
    {
      float freq = i * freq_step;
      float magnitude = sqrtf(g_fft_out[i].r * g_fft_out[i].r + g_fft_out[i].i * g_fft_out[i].i);
      printf("%.2f | %.6f\n", freq, magnitude);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  struct sensor_accel acc_data;
  float samples[NUM_SAMPLES];
  int sample_count = 0;

  printf("MPU60x0 Accelerometer FFT Test\n");
  printf("Sample Rate: %d Hz\n", SAMPLE_FREQ);
  printf("Number of samples: %d\n", NUM_SAMPLES);
  printf("Frequency Resolution: %.3f Hz\n", (float)SAMPLE_FREQ/NUM_SAMPLES);

  fd = open("/dev/imu0", O_RDONLY);
  if (fd < 0)
    {
      printf("Failed to open imu0\n");
      return EXIT_FAILURE;
    }

  while (1)
    {
      read_mpu6050(fd, &acc_data);

      /* Store X-axis acceleration */
      samples[sample_count] = acc_data.x;
      sample_count++;

      /* When we have enough samples, perform FFT */
      if (sample_count >= NUM_SAMPLES)
        {
          process_fft(samples, NUM_SAMPLES);
          sample_count = 0;
        }

      usleep(SAMPLE_RATE_MS * 1000);
    }

  /* Never reaches here */
  close(fd);
  return EXIT_SUCCESS;
}
