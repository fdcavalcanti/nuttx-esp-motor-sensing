/***************************************************************************
 * apps/motor_sensing/qencoder_main.c
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
#include <stdbool.h>

#include <nuttx/sensors/qencoder.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QE_DEVPATH       "/dev/qe0"
#define TASK_DELAY_MS    100
#define BASE_PPR         11    /* Base encoder PPR */
#define GEAR_RATIO       34      /* Gear reduction ratio */
#define PULSES_PER_REV   (BASE_PPR * GEAR_RATIO)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_should_exit = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_help(void)
{
  printf("Usage:\n");
  printf("  qencoder [options]\n");
  printf("Options:\n");
  printf("  -r         : Reset counter\n");
  printf("  -x         : Exit program\n");
  printf("  -h         : Show this help message\n");
}

static float calculate_rpm(int32_t pulses, uint32_t time_ms)
{
  /* Convert encoder pulses to RPM:
   * RPM = (pulses/4 / PULSES_PER_REV) * (60000 / time_ms)
   * Note: divide by 4 because driver uses X4 encoding by default
   */

  return ((float)(pulses/4) * 60000.0f) / ((float)PULSES_PER_REV * (float)time_ms);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret;
  int position;
  float rpm;

  /* Check arguments */

  if (argc > 1)
    {
      if (!strcmp(argv[1], "-h"))
        {
          show_help();
          return OK;
        }
      else if (!strcmp(argv[1], "-r"))
        {
          fd = open(QE_DEVPATH, O_RDWR);
          if (fd < 0)
            {
              printf("Failed to open encoder device\n");
              return ERROR;
            }

          ret = ioctl(fd, QEIOC_RESET, 0);
          if (ret < 0)
            {
              printf("Failed to reset encoder: %d\n", ret);
              close(fd);
              return ERROR;
            }

          close(fd);
          return OK;
        }
      else if (!strcmp(argv[1], "-x"))
        {
          g_should_exit = true;
          return OK;
        }
      else
        {
          printf("Invalid argument\n");
          show_help();
          return ERROR;
        }
    }

  /* Open encoder device */

  fd = open(QE_DEVPATH, O_RDWR);
  if (fd < 0)
    {
      printf("Failed to open encoder device\n");
      return ERROR;
    }

  /* Reset counter on start */
  ret = ioctl(fd, QEIOC_RESET, 0);
  if (ret < 0)
    {
      printf("Failed to reset encoder: %d\n", ret);
      close(fd);
      return ERROR;
    }

  printf("Reading encoder...\n");
  printf("Sample time: %d ms\n", TASK_DELAY_MS);

  /* Main loop - read encoder and calculate speed */

  while (!g_should_exit)
    {
      ret = ioctl(fd, QEIOC_POSITION, (unsigned long)((uintptr_t)&position));
      if (ret < 0)
        {
          printf("Failed to read position: %d\n", ret);
          break;
        }

      /* Reset counter to avoid overflow */
      ret = ioctl(fd, QEIOC_RESET, 0);
      if (ret < 0)
        {
          printf("Failed to reset encoder: %d\n", ret);
          break;
        }

      /* Calculate speed based on position */
      rpm = calculate_rpm(position, TASK_DELAY_MS);
      printf("Pulses: %d, Speed: %.2f RPM\n", position, rpm);

      usleep(TASK_DELAY_MS * 1000);
    }

  close(fd);
  return ret;
}
