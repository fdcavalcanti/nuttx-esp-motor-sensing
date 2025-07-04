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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_LOW_MASK    0xFF00
#define REG_HIGH_MASK   0x00FF
#define MPU6050_AFS_SEL 4096.0f   /* Accel scale factor */
#define SAMPLE_RATE_MS  20        /* 50 Hz sample rate */
#define TCP_PORT        5000      /* TCP port to send data */
#define MAX_MSG_SIZE    64        /* Maximum message size */
#define MAX_CLIENTS     1         /* Maximum number of clients */

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
  struct sensor_accel acc_data = {0};
  char msg_buffer[MAX_MSG_SIZE];
  
  /* Socket variables */
  int server_fd, client_fd;
  struct sockaddr_in server_addr, client_addr;
  socklen_t client_len = sizeof(client_addr);
  int opt = 1;
  
  printf("MPU60x0 Accelerometer Test\n");
  printf("Sample Rate: %d ms (%d Hz)\n", SAMPLE_RATE_MS, 1000/SAMPLE_RATE_MS);
  printf("TCP server starting on port %d\n", TCP_PORT);

  /* Open IMU device */
  fd = open("/dev/imu0", O_RDONLY);
  if (fd < 0)
    {
      printf("Failed to open imu0\n");
      return EXIT_FAILURE;
    }

  /* Create TCP socket */
  server_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (server_fd < 0)
    {
      printf("Failed to create socket\n");
      close(fd);
      return EXIT_FAILURE;
    }

  /* Set socket options */
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR,
                 &opt, sizeof(opt)) < 0)
    {
      printf("Failed to set socket options\n");
      close(fd);
      close(server_fd);
      return EXIT_FAILURE;
    }

  /* Configure server address */
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(TCP_PORT);

  /* Bind socket */
  if (bind(server_fd, (struct sockaddr *)&server_addr,
           sizeof(server_addr)) < 0)
    {
      printf("Failed to bind socket\n");
      close(fd);
      close(server_fd);
      return EXIT_FAILURE;
    }

  /* Listen for connections */
  if (listen(server_fd, MAX_CLIENTS) < 0)
    {
      printf("Failed to listen\n");
      close(fd);
      close(server_fd);
      return EXIT_FAILURE;
    }

  printf("Waiting for client connection...\n");
  
  /* Accept client connection */
  client_fd = accept(server_fd, (struct sockaddr *)&client_addr,
                    &client_len);
  if (client_fd < 0)
    {
      printf("Failed to accept client\n");
      close(fd);
      close(server_fd);
      return EXIT_FAILURE;
    }

  printf("Client connected\n");

  while (1)
    {
      read_mpu6050(fd, &acc_data);

      /* Format data as string with newline */
      snprintf(msg_buffer, MAX_MSG_SIZE, "%.3f,%.3f,%.3f\n",
               acc_data.x, acc_data.y, acc_data.z);

      /* Send data over TCP */
      ssize_t bytes_sent = send(client_fd, msg_buffer,
                               strlen(msg_buffer), 0);
      
      /* Check if client disconnected */
      if (bytes_sent <= 0)
        {
          printf("Client disconnected, waiting for new connection...\n");
          close(client_fd);
          
          /* Wait for new client */
          client_fd = accept(server_fd, (struct sockaddr *)&client_addr,
                           &client_len);
          if (client_fd < 0)
            {
              printf("Failed to accept new client\n");
              break;
            }
          printf("New client connected\n");
          continue;
        }

      printf("Accel (g): X=%.3f Y=%.3f Z=%.3f\n",
             acc_data.x, acc_data.y, acc_data.z);

      usleep(SAMPLE_RATE_MS * 1000);
    }

  /* Cleanup */
  close(fd);
  close(client_fd);
  close(server_fd);
  return EXIT_SUCCESS;
}
