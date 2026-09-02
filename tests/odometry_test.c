#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "odometry.h"

static void expect_near(const char *name, float actual, float expected, float tolerance)
{
  if(fabsf(actual - expected) > tolerance)
  {
    fprintf(stderr, "%s actual=%.3f expected=%.3f\n", name, actual, expected);
    exit(1);
  }
}

static void test_straight_and_reverse(void)
{
  odometry_t odom;
  odometry_init();
  odometry_update(0, 0, 0.0f, 0.01f);
  odometry_update(2640, 2640, 0.0f, 0.01f);
  odometry_get(&odom);
  expect_near("forward x", odom.x_mm, 267.035f, 0.01f);
  odometry_update(0, 0, 0.0f, 0.01f);
  odometry_get(&odom);
  expect_near("reverse x", odom.x_mm, 0.0f, 0.01f);
}

static void test_encoder_constrains_gyro_bias(void)
{
  odometry_t odom;
  unsigned int index;
  odometry_init();
  for(index = 0U; index < 2000U; index++) odometry_update(0, 0, 0.1f, 0.01f);
  odometry_get(&odom);
  expect_near("bias constrained", odom.yaw_deg, 0.20f, 0.03f);
}

static void test_encoder_spike_rejected(void)
{
  odometry_t odom;
  odometry_init();
  odometry_update(0, 0, 0.0f, 0.01f);
  odometry_update(-3882, 3882, 0.0f, 0.01f);
  odometry_get(&odom);
  if(odom.encoder_rejected == 0U) exit(1);
  expect_near("spike yaw", odom.yaw_deg, 0.0f, 0.01f);
}

int main(void)
{
  test_straight_and_reverse();
  test_encoder_constrains_gyro_bias();
  test_encoder_spike_rejected();
  puts("ODOMETRY_TEST_PASS");
  return 0;
}
