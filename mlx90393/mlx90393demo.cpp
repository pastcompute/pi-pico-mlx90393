// Copyright 2022 Andrew McDonnell - updated to support the Raspberry Pi Pico SDK
// SPDX-License-Identifier: MIT

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/platform.h"
#include "pico/binary_info.h"
#include "MLX90393.h"

#define delayMicroseconds(us) busy_wait_us_32(us)
#define delay(ms) busy_wait_us(ms * 1000)

MLX90393 mlx;

static void setupI2c(uint8_t sda, uint8_t scl) {
  i2c_init(i2c1, 1000 * 1000);
  gpio_set_function(sda, GPIO_FUNC_I2C);
  gpio_set_function(scl, GPIO_FUNC_I2C);
  gpio_pull_up(sda);
  gpio_pull_up(scl);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(sda, scl, GPIO_FUNC_I2C));
}

int main() {
  auto t0 = to_ms_since_boot(get_absolute_time());
  stdio_init_all();
  setupI2c(6,7);
  mlx.begin(0,0,-1,i2c1);
  mlx.reset(); // beware, this changes defaults from begin()
  mlx.setGainSel(7);
  mlx.setResolution(0, 0, 0);
  mlx.setOverSampling(3); // increases mindelay
  mlx.setTemperatureCompensation(0);
  mlx.setDigitalFiltering(5); // reduces mindelay
  auto minDelay = mlx.convDelayMillis();
  printf("MLX90393.minDelay=%d\n", minDelay);

  auto tloop = to_ms_since_boot(get_absolute_time());
  while(true) {
    auto status = mlx.nop();
    if ( MLX90393::STATUS_ERROR == status) {
      panic("Error reading MLX90393 NOP");
    }
    MLX90393::txyzRaw raw;
    status = mlx.startMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG);

    if (status & MLX90393::ERROR_BIT) {
      printf("Error starting MLX90393 measure\n");
      delay(5000);
      continue;
    }

    if (minDelay < 1) { delayMicroseconds(600); } else { delay(minDelay); }

    status = mlx.readMeasurement(MLX90393::X_FLAG | MLX90393::Y_FLAG | MLX90393::Z_FLAG | MLX90393::T_FLAG, raw);
    if (status & MLX90393::ERROR_BIT) {
      printf("Error reading MLX90393 measure\n");
      delay(5000);
      continue;
    }

    const MLX90393::txyz values = mlx.convertRaw(raw);

    auto now = to_ms_since_boot(get_absolute_time());
    auto tn = (now - t0);
    auto dt = now - tloop;
    tloop = now;

    printf("%lu,%lu,%d,%d,%d,%d\n", tn, minDelay, dt, (int)values.x, (int)values.y, (int)values.z, (int)values.t);
  }
}
