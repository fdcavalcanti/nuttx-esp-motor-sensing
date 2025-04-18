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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MOTOR_DEVPATH "/dev/motor0"
#define SPEED_FILE "/mnt/mspeed"
#define TASK_DELAY_US 100000
#define SPEED_BUF_SIZE 32

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
  printf("Options:\n");
  printf("  -s <speed> : Set initial motor speed (0.0 to 1.0)\n");
  printf("  -x         : Stop motor\n");
  printf("  -h         : Show this help message\n");
  printf("\nTo change speed while running:\n");
  printf("  echo \"<speed>\" > %s\n", SPEED_FILE);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int motor_set_speed(int fd, float speed)
{
  int ret;

  if (speed < 0.0 || speed > 1.0)
    {
      printf("Error: Speed must be between 0.0 and 1.0\n");
      return ERROR;
    }

  printf("Setting motor speed to: %f\n", speed);

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

  ret = ioctl(fd, MTRIOC_START, 0);
  if (ret < 0)
    {
      printf("Failed to start motor: %d\n", ret);
      return ret;
    }

  return OK;
}

static int motor_stop(int fd)
{
  int ret;

  printf("Stopping motor\n");

  ret = ioctl(fd, MTRIOC_STOP, 0);
  if (ret < 0)
    {
      printf("Failed to stop motor: %d\n", ret);
      return ret;
    }

  g_should_exit = true;
  return OK;
}

static int create_speed_file(void)
{
  int fd;

  fd = creat(SPEED_FILE, 0666);
  if (fd < 0)
    {
      printf("Failed to create speed file %s: %d\n", SPEED_FILE, fd);
      return ERROR;
    }

  close(fd);
  return OK;
}

static int check_speed_update(int motor_fd)
{
  int fd;
  char buf[SPEED_BUF_SIZE];
  ssize_t nbytes;
  float new_speed;

  fd = open(SPEED_FILE, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      return OK;
    }

  nbytes = read(fd, buf, sizeof(buf) - 1);
  close(fd);

  if (nbytes <= 0)
    {
      return OK;
    }

  buf[nbytes] = '\0';
  new_speed = atof(buf);

  /* Clear the speed file */
  fd = open(SPEED_FILE, O_WRONLY | O_TRUNC);
  if (fd >= 0)
    {
      close(fd);
    }

  return motor_set_speed(motor_fd, new_speed);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int fd;
  int ret = OK;
  float speed;

  /* Check arguments */

  if (argc < 2)
    {
      show_help();
      return ERROR;
    }

  /* Create speed control file */
  ret = create_speed_file();
  if (ret < 0)
    {
      return ERROR;
    }

  /* Open motor device */

  fd = open(MOTOR_DEVPATH, O_RDWR);
  if (fd < 0)
    {
      printf("Failed to open motor device\n");
      return ERROR;
    }

  /* Set motor limits */

  ret = ioctl(fd, MTRIOC_SET_LIMITS, &limits);
  if (ret < 0)
    {
      printf("Failed to set motor limits\n");
      close(fd);
      return ERROR;
    }

  /* Parse command line arguments */

  if (!strcmp(argv[1], "-h"))
    {
      show_help();
      close(fd);
      return OK;
    }
  else if (!strcmp(argv[1], "-x"))
    {
      ret = motor_stop(fd);
    }
  else if (!strcmp(argv[1], "-s") && argc == 3)
    {
      speed = atof(argv[2]);
      ret = motor_set_speed(fd, speed);
      if (ret < 0)
        {
          close(fd);
          return ret;
        }
    }
  else
    {
      printf("Invalid arguments\n");
      show_help();
      close(fd);
      return ERROR;
    }

  /* Keep task running with delay and check for speed updates */
  while (!g_should_exit)
    {
      check_speed_update(fd);
      usleep(TASK_DELAY_US);
    }

  close(fd);
  unlink(SPEED_FILE);
  return ret;
}
