/***************************************************************************
 * apps/motor_sensing/motor_sensing_main.c
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
#include <inttypes.h>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <nuttx/motor/motor.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/sensors/qencoder.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MOTOR_DEVPATH "/dev/motor0"
#define ADC_DEVPATH "/dev/adc0"
#define QE_DEVPATH "/dev/qe0"
#define ADC_MIN_THRESHOLD 100
#define ADC_MAX_THRESHOLD 2500
#define TASK_DELAY_MS 100
#define BASE_PPR 11        /* Base encoder PPR */
#define GEAR_RATIO 34      /* Gear reduction ratio */
#define PULSES_PER_REV (BASE_PPR * GEAR_RATIO)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* At least one limit must be set to allow motor start.
 * Value 1 represents 100% or max voltage.
 */

static struct motor_limits_s limits =
{
  .speed = 1,
};

static struct motor_params_s params;
static bool g_should_exit = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_help(void)
{
  printf("Usage:\n");
  printf("  motor_sensing [options]\n");
  printf("\nDescription:\n");
  printf("  Controls motor speed based on ADC readings from channel 0.\n");
  printf("  ADC values < %d: Motor stops\n", ADC_MIN_THRESHOLD);
  printf("  ADC values > %d: Full speed\n", ADC_MAX_THRESHOLD);
  printf("  Values in between are mapped linearly to speed (0.0 to 1.0)\n");
  printf("\nOptions:\n");
  printf("  -h         : Show this help message\n");
}

static float calculate_rpm(int32_t pulses, uint32_t time_ms)
{
  /* Convert encoder pulses to RPM:
   * RPM = (pulses/4 / PULSES_PER_REV) * (60000 / time_ms)
   * Note: divide by 4 because driver uses X4 encoding by default
   */

  return ((float)(pulses / 4.0f) * 60000.0f) / ((float)PULSES_PER_REV * (float)time_ms);
}

static int motor_set_speed(int fd, float speed)
{
  int ret;
  struct motor_state_s state;

  if (speed < 0.0 || speed > 1.0)
    {
      printf("Error: Speed must be between 0.0 and 1.0\n");
      return ERROR;
    }

  /* Get current motor state */
  ret = ioctl(fd, MTRIOC_GET_STATE, (unsigned long)&state);
  if (ret < 0)
    {
      printf("Failed to get motor state: %d\n", ret);
      return ret;
    }

  ret = ioctl(fd, MTRIOC_SET_MODE, MOTOR_OPMODE_SPEED);
  if (ret < 0)
    {
      printf("Failed to set speed mode: %d\n", ret);
      return ret;
    }

  params.speed = speed;
  ret = ioctl(fd, MTRIOC_SET_PARAMS, &params);
  if (ret < 0)
    {
      printf("Failed to set parameters: %d\n", ret);
      return ret;
    }

  /* Only start if not already running */
  if (state.state != MOTOR_STATE_RUN)
    {
      ret = ioctl(fd, MTRIOC_START, 0);
      if (ret < 0)
        {
          printf("Failed to start motor: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

static int check_speed_update(int adc_fd, float *speed)
{
  int ret;
  struct adc_msg_s sample;
  size_t readsize;
  ssize_t nbytes;

  if (speed == NULL)
    {
      return ERROR;
    }

  /* Trigger ADC conversion */
  ret = ioctl(adc_fd, ANIOC_TRIGGER, 0);
  if (ret < 0)
    {
      printf("Failed to trigger ADC: %d\n", errno);
      return ERROR;
    }

  /* Read ADC value */
  readsize = sizeof(struct adc_msg_s);
  nbytes = read(adc_fd, &sample, readsize);
  if (nbytes <= 0)
    {
      printf("Failed to read ADC: %d\n", errno);
      return ERROR;
    }

  /* Apply thresholds and map ADC value to speed */
  if (sample.am_data < ADC_MIN_THRESHOLD)
    {
      *speed = 0.0;
    }
  else if (sample.am_data > ADC_MAX_THRESHOLD)
    {
      *speed = 1.0;
    }
  else
    {
      /* Linear mapping from ADC range to speed range */
      *speed = (float)(sample.am_data - ADC_MIN_THRESHOLD) /
               (float)(ADC_MAX_THRESHOLD - ADC_MIN_THRESHOLD);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int motor_fd;
  int adc_fd;
  int qe_fd;
  int ret = OK;
  float speed;
  int position;
  float rpm;

  /* Parse command line arguments if provided */
  if (argc > 1)
    {
      if (!strcmp(argv[1], "-h"))
        {
          show_help();
          return OK;
        }
      else
        {
          printf("Invalid arguments\n");
          show_help();
          return ERROR;
        }
    }

  /* Open motor device */
  motor_fd = open(MOTOR_DEVPATH, O_RDWR);
  if (motor_fd < 0)
    {
      printf("Failed to open motor device\n");
      return ERROR;
    }

  /* Open ADC device */
  adc_fd = open(ADC_DEVPATH, O_RDWR);
  if (adc_fd < 0)
    {
      printf("Failed to open ADC device: %d\n", errno);
      close(motor_fd);
      return ERROR;
    }

  /* Open encoder device */
  qe_fd = open(QE_DEVPATH, O_RDWR);
  if (qe_fd < 0)
    {
      printf("Failed to open encoder device: %d\n", errno);
      close(motor_fd);
      close(adc_fd);
      return ERROR;
    }

  /* Set motor limits */
  ret = ioctl(motor_fd, MTRIOC_SET_LIMITS, &limits);
  if (ret < 0)
    {
      printf("Failed to set motor limits\n");
      close(motor_fd);
      close(adc_fd);
      close(qe_fd);
      return ERROR;
    }

  /* Reset encoder counter */
  ret = ioctl(qe_fd, QEIOC_RESET, 0);
  if (ret < 0)
    {
      printf("Failed to reset encoder: %d\n", ret);
      close(motor_fd);
      close(adc_fd);
      close(qe_fd);
      return ERROR;
    }

  /* Keep task running with delay and check for speed updates */
  printf("Reading ADC and controlling motor...\n");
  printf("Sample time: %d ms\n", TASK_DELAY_MS);
  printf("ADC range: %d to %d\n", ADC_MIN_THRESHOLD, ADC_MAX_THRESHOLD);
  printf("Encoder PPR: %d\n", PULSES_PER_REV);

  while (!g_should_exit)
    {
      /* Get commanded speed from ADC */
      ret = check_speed_update(adc_fd, &speed);
      if (ret == OK)
        {
          ret = motor_set_speed(motor_fd, speed);
          if (ret != OK)
            {
              printf("Failed to set motor speed\n");
              break;
            }
        }
      else
        {
          printf("Failed to update speed from ADC\n");
          break;
        }

      /* Read encoder position */
      ret = ioctl(qe_fd, QEIOC_POSITION, (unsigned long)((uintptr_t)&position));
      if (ret < 0)
        {
          printf("Failed to read position: %d\n", ret);
          break;
        }

      /* Reset counter to avoid overflow */
      ret = ioctl(qe_fd, QEIOC_RESET, 0);
      if (ret < 0)
        {
          printf("Failed to reset encoder: %d\n", ret);
          break;
        }

      /* Calculate and display speeds */
      rpm = calculate_rpm(position, TASK_DELAY_MS);
      printf("Command: %.2f%%, Speed: %.2f RPM\n", speed * 100.0f, rpm);

      usleep(TASK_DELAY_MS * 1000);
    }

  close(motor_fd);
  close(adc_fd);
  close(qe_fd);
  return ret;
}
