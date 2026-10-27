#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board_config.h"
#include "canfd_driver.h"
#include "chassis_protocol.h"

static canfd_frame_t frames[16];
static canfd_frame_t sent;
static uint8_t read_index;
static uint8_t write_index;
static uint32_t rx_order;
static const uint8_t motion_bytes[16] = {
  0x01, 0x00, 0x78, 0x00, 0x2c, 0x01, 0x00, 0x00,
  0xf4, 0x01, 0xe8, 0x03, 0x00, 0x00, 0x00, 0x00
};

void canfd_driver_init(void) { read_index = write_index = 0U; rx_order = 0U; }
void canfd_driver_poll(uint32_t now_ms) { (void)now_ms; }
uint32_t canfd_driver_rx_order(void) { return rx_order; }
uint8_t canfd_driver_healthy(void) { return 1U; }
uint8_t canfd_driver_take(canfd_frame_t *frame)
{
  if(read_index == write_index) return 0U;
  *frame = frames[read_index++];
  return 1U;
}
uint8_t canfd_driver_send(uint32_t id, const uint8_t *data, uint8_t size)
{
  sent.id = id;
  sent.size = size;
  memcpy(sent.data, data, size);
  return 1U;
}

static void check(int value, const char *message)
{
  if(!value) { fprintf(stderr, "%s\n", message); exit(1); }
}

static canfd_frame_t *enqueue(uint32_t id, uint8_t size, uint32_t receive_ms)
{
  canfd_frame_t *frame;
  check(write_index < 16U, "test queue overflow");
  frame = &frames[write_index++];
  memset(frame, 0, sizeof(*frame));
  frame->id = id;
  frame->size = size;
  frame->receive_ms = receive_ms;
  frame->order = ++rx_order;
  return frame;
}

static void command(chassis_command_type_t type, uint16_t sequence)
{
  canfd_frame_t *frame = enqueue(BOARD_CAN_ID_CONTROL, 8U, 0U);
  frame->data[0] = (uint8_t)type;
  frame->data[2] = (uint8_t)sequence;
  frame->data[3] = (uint8_t)(sequence >> 8U);
}

static canfd_frame_t *motion(uint16_t sequence, uint32_t receive_ms)
{
  canfd_frame_t *frame = enqueue(BOARD_CAN_ID_REFERENCE, 16U, receive_ms);
  memcpy(frame->data, motion_bytes, sizeof(motion_bytes));
  frame->data[0] = (uint8_t)sequence;
  frame->data[1] = (uint8_t)(sequence >> 8U);
  return frame;
}

static void expect_command(chassis_command_type_t type, uint16_t sequence)
{
  chassis_command_t value;
  chassis_protocol_poll(0U);
  check(chassis_protocol_take_command(&value), "command missing");
  check((value.type == type) && (value.sequence == sequence), "wrong command won");
  check(!chassis_protocol_take_command(&value), "command consumed twice");
}

static void test_stop_precedence(void)
{
  chassis_command_t value;
  chassis_protocol_init();
  command(CHASSIS_CMD_STOP, 100U);
  command(CHASSIS_CMD_ENABLE, 101U);
  expect_command(CHASSIS_CMD_STOP, 100U);
  command(CHASSIS_CMD_STOP, 102U);
  command(CHASSIS_CMD_FAULT_RESET, 103U);
  expect_command(CHASSIS_CMD_STOP, 102U);
  command(CHASSIS_CMD_DISABLE, 104U);
  command(CHASSIS_CMD_ENABLE, 105U);
  expect_command(CHASSIS_CMD_DISABLE, 104U);
  command(CHASSIS_CMD_ESTOP, 1U);
  command(CHASSIS_CMD_STOP, 106U);
  command(CHASSIS_CMD_FAULT_RESET, 107U);
  expect_command(CHASSIS_CMD_ESTOP, 1U);
  command(CHASSIS_CMD_ENABLE, 106U);
  chassis_protocol_poll(0U);
  check(!chassis_protocol_take_command(&value), "stale ESTOP rolled history back");
  command(CHASSIS_CMD_STOP, 1U);
  expect_command(CHASSIS_CMD_STOP, 1U);
  command(CHASSIS_CMD_DISABLE, 1U);
  expect_command(CHASSIS_CMD_DISABLE, 1U);
}

static void test_command_zero_and_wrap(void)
{
  chassis_command_t value;
  chassis_protocol_init();
  command(CHASSIS_CMD_ENABLE, 0U);
  expect_command(CHASSIS_CMD_ENABLE, 0U);
  command(CHASSIS_CMD_ENABLE, 0U);
  chassis_protocol_poll(0U);
  check(!chassis_protocol_take_command(&value), "sequence zero accepted twice");
  chassis_protocol_init();
  command(CHASSIS_CMD_ENABLE, 65535U);
  expect_command(CHASSIS_CMD_ENABLE, 65535U);
  command(CHASSIS_CMD_RESET_ODOMETRY, 0U);
  expect_command(CHASSIS_CMD_RESET_ODOMETRY, 0U);
  command(CHASSIS_CMD_ENABLE, 65535U);
  chassis_protocol_poll(0U);
  check(!chassis_protocol_take_command(&value), "pre-wrap command replayed");
  command(CHASSIS_CMD_ENABLE, 0x1234U);
  expect_command(CHASSIS_CMD_ENABLE, 0x1234U);
}

static void test_reference_history_and_barrier(void)
{
  drive_reference_t reference;
  chassis_protocol_init();
  motion(100U, 0U);
  chassis_protocol_poll(0U);
  check(chassis_protocol_get_reference(&reference), "first reference missing");
  chassis_protocol_clear_reference();
  motion(99U, 10U);
  chassis_protocol_poll(10U);
  check(!chassis_protocol_get_reference(&reference), "clear forgot sequence");
  motion(101U, 20U);
  chassis_protocol_poll(20U);
  check(chassis_protocol_get_reference(&reference), "new reference rejected");
  motion(102U, 30U);
  chassis_protocol_clear_reference();
  chassis_protocol_poll(30U);
  check(!chassis_protocol_get_reference(&reference), "pre-enable queued reference accepted");
  motion(103U, 40U);
  chassis_protocol_poll(40U);
  check(chassis_protocol_get_reference(&reference), "post-enable reference rejected");
  chassis_protocol_init();
  rx_order = UINT32_MAX - 1U;
  chassis_protocol_clear_reference();
  motion(65535U, 0U);
  chassis_protocol_poll(0U);
  check(chassis_protocol_get_reference(&reference), "reference before wrap missing");
  chassis_protocol_clear_reference();
  motion(0U, 10U);
  chassis_protocol_poll(10U);
  check(chassis_protocol_get_reference(&reference), "order/sequence wrap rejected");
}

static void test_reference_does_not_expire(void)
{
  drive_reference_t reference;
  chassis_protocol_init();
  motion(1U, 100U);
  chassis_protocol_poll(1000000U);
  check(chassis_protocol_get_reference(&reference), "queued target was expired locally");
  chassis_protocol_poll(UINT32_MAX);
  check(chassis_protocol_get_reference(&reference), "target did not hold without refresh");
  chassis_protocol_clear_reference();
  check(!chassis_protocol_get_reference(&reference), "explicit clear did not revoke target");
}

static void test_reference_fields(void)
{
  const struct { uint8_t offset; uint16_t value; } invalid[] = {
    {12U, 1U}, {14U, 1U}
  };
  drive_reference_t reference;
  canfd_frame_t *frame;
  size_t index;
  chassis_protocol_init();
  motion(1U, 0U);
  chassis_protocol_poll(0U);
  check(chassis_protocol_get_reference(&reference), "golden reference missing");
  check(reference.linear_mm_s == 300 && reference.angular_mrad_s == 0 &&
        reference.accel_mm_s2 == 500U && reference.decel_mm_s2 == 1000U,
        "golden reference decoding changed");
  for(index = 0U; index < sizeof(invalid) / sizeof(invalid[0]); index++)
  {
    chassis_protocol_init();
    frame = motion(1U, 0U);
    frame->data[invalid[index].offset] = (uint8_t)invalid[index].value;
    frame->data[invalid[index].offset + 1U] = (uint8_t)(invalid[index].value >> 8U);
    chassis_protocol_poll(0U);
    check(!chassis_protocol_get_reference(&reference), "out-of-range field accepted");
    motion(1U, 1U);
    chassis_protocol_poll(1U);
    check(chassis_protocol_get_reference(&reference), "invalid field consumed sequence");
  }
  chassis_protocol_init();
  frame = motion(1U, 0U);
  memset(&frame->data[4], 0, 8U);
  chassis_protocol_poll(0U);
  check(chassis_protocol_get_reference(&reference), "all-zero HOLD rejected");
  chassis_protocol_init();
  frame = motion(1U, 0U);
  frame->size = 12U;
  chassis_protocol_poll(0U);
  check(!chassis_protocol_get_reference(&reference), "short reference accepted");
}

static void test_upper_owned_ranges(void)
{
  const uint8_t offsets[] = {2U, 4U, 6U, 8U, 10U};
  drive_reference_t reference;
  size_t i;
  for(i = 0U; i < sizeof(offsets); i++)
  {
    canfd_frame_t *frame;
    chassis_protocol_init();
    frame = motion(1U, 0U);
    frame->data[offsets[i]] = 0xffU;
    frame->data[offsets[i] + 1U] = 0x7fU;
    chassis_protocol_poll(5000U);
    check(chassis_protocol_get_reference(&reference), "local range gate remains");
  }
  chassis_protocol_init();
  motion(1U, 0U)->data[2] = 0U;
  chassis_protocol_poll(0U);
  check(chassis_protocol_get_reference(&reference), "zero TTL is still locally enforced");
}

static void test_output_golden_bytes(void)
{
  const chassis_status_t status = {2U, 3U, 4U, 5U, 0x1234U, 6U, -100, 200,
                                  24000U, 100U, 200U};
  const odometry_t odometry = {-100.0f, 200.0f, -90.0f, 1U, 0U};
  const uint8_t status_bytes[24] = {2, 3, 4, 5, 0x34, 0x12, 6, 0,
    0x78, 0x56, 0x34, 0x12, 0x9c, 0xff, 0xc8, 0, 0xc0, 0x5d, 100, 0, 200, 0, 0, 0};
  const uint8_t odometry_bytes[20] = {0x9c, 0xff, 0xff, 0xff, 0xc8, 0, 0, 0,
    0xd8, 0xdc, 0xff, 0xff, 0xd4, 0xfe, 0x90, 1, 0x78, 0x56, 0x34, 0x12};
  chassis_protocol_send_status(&status, 0x12345678UL);
  check(sent.id == 0x181U && sent.size == 24U &&
        memcmp(sent.data, status_bytes, 24U) == 0, "status wire bytes changed");
  chassis_protocol_send_odometry(&odometry, -300, 400, 0x12345678UL);
  check(sent.id == 0x381U && sent.size == 20U &&
        memcmp(sent.data, odometry_bytes, 20U) == 0, "odometry wire bytes changed");
}

int main(void)
{
  test_stop_precedence();
  test_command_zero_and_wrap();
  test_reference_history_and_barrier();
  test_reference_does_not_expire();
  test_upper_owned_ranges();
  test_reference_fields();
  test_output_golden_bytes();
  puts("CHASSIS_PROTOCOL_TEST_PASS");
  return 0;
}
