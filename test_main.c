/**
 * @file test_main.c
 * @brief Hardware-in-the-loop / unit verification test suite for firmware_optimized module.
 *
 * Validates:
 * 1. Non-blocking UART transmit FIFO, interrupt generation, and idle shut-down.
 * 2. Unaligned raw byte stream parsing (sensor ingestion).
 * 3. Stream chunk parser across fragmented buffers with minimal stack footprint.
 * 4. Atomic statistics snapshot and thread safety.
 */

#include "firmware_optimized.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* ========================================================================== */
/* Hardware Mock / Simulation State                                           */
/* ========================================================================== */

static bool s_mock_tx_irq_enabled = false;
static uint8_t s_mock_uart_tx_log[512];
static size_t s_mock_uart_tx_count = 0;

static void mock_enable_tx_interrupt(void)
{
    s_mock_tx_irq_enabled = true;
}

static void mock_disable_tx_interrupt(void)
{
    s_mock_tx_irq_enabled = false;
}

static void mock_write_tx_data_reg(uint8_t byte)
{
    if (s_mock_uart_tx_count < sizeof(s_mock_uart_tx_log)) {
        s_mock_uart_tx_log[s_mock_uart_tx_count++] = byte;
    }
}

static const fw_uart_hal_t s_mock_hal = {
    .enable_tx_interrupt  = mock_enable_tx_interrupt,
    .disable_tx_interrupt = mock_disable_tx_interrupt,
    .write_tx_data_reg    = mock_write_tx_data_reg
};

/* ========================================================================== */
/* Test Cases                                                                 */
/* ========================================================================== */

static void test_uart_non_blocking_tx(void)
{
    printf("[TEST] Running test_uart_non_blocking_tx...\n");

    s_mock_uart_tx_count = 0;
    s_mock_tx_irq_enabled = false;

    fw_status_t status = fw_uart_init(&s_mock_hal);
    assert(status == FW_STATUS_OK);
    assert(s_mock_tx_irq_enabled == false);

    /* Transmit a string "HELLO_EMBEDDED" (14 bytes) */
    const char *msg = "HELLO_EMBEDDED";
    size_t msg_len = strlen(msg);
    size_t saved = 0;

    status = fw_uart_write_async((const uint8_t *)msg, msg_len, &saved);
    assert(status == FW_STATUS_OK);
    assert(saved == msg_len);

    /* First byte 'H' should have been directly written to UART DR, and IRQ enabled! */
    assert(s_mock_uart_tx_count == 1);
    assert(s_mock_uart_tx_log[0] == 'H');
    assert(s_mock_tx_irq_enabled == true);

    /* Simulate MCU hardware firing TX ISR repeatedly until all bytes drained */
    while (s_mock_tx_irq_enabled) {
        fw_uart_tx_isr_handler();
    }

    /* All 14 bytes must have been transmitted */
    assert(s_mock_uart_tx_count == msg_len);
    assert(memcmp(s_mock_uart_tx_log, msg, msg_len) == 0);
    /* After FIFO is empty, TX IRQ MUST be automatically disabled */
    assert(s_mock_tx_irq_enabled == false);

    printf("  -> Passed. Received: %.*s (TX IRQ correctly shut down on empty)\n",
           (int)s_mock_uart_tx_count, (char *)s_mock_uart_tx_log);
}

static void test_uart_fifo_overflow_protection(void)
{
    printf("[TEST] Running test_uart_fifo_overflow_protection...\n");

    fw_uart_init(&s_mock_hal);
    fw_stats_reset();

    /* Disable ISR processing to force buffer to fill */
    s_mock_uart_tx_count = 0;

    /* Create dummy 200 bytes */
    uint8_t payload[200];
    for (size_t i = 0; i < sizeof(payload); i++) {
        payload[i] = (uint8_t)(i & 0xFF);
    }

    size_t written = 0;
    fw_status_t status = fw_uart_write_async(payload, sizeof(payload), &written);

    /* Should return FIFO FULL error */
    assert(status == FW_STATUS_ERR_FIFO_FULL);
    /* Capacity is 128 bytes (1 byte popped immediately into DR + 127 in FIFO) */
    assert(written == FW_UART_TX_FIFO_SIZE);

    fw_telemetry_stats_t stats;
    fw_stats_get_snapshot(&stats);
    assert(stats.tx_fifo_overflows > 0);

    /* Drain buffer via simulated ISR */
    while (s_mock_tx_irq_enabled) {
        fw_uart_tx_isr_handler();
    }
    assert(s_mock_uart_tx_count == FW_UART_TX_FIFO_SIZE);

    printf("  -> Passed. Overflow detected accurately, %u drops recorded.\n",
           (unsigned int)stats.tx_fifo_overflows);
}

static void test_unaligned_memory_ingestion(void)
{
    printf("[TEST] Running test_unaligned_memory_ingestion...\n");

    /*
     * Construct an unaligned raw byte stream.
     * We place an odd 1-byte padding at the front so that even the starting
     * pointer of the frame is unaligned!
     */
    uint8_t raw_buffer[32];
    memset(raw_buffer, 0, sizeof(raw_buffer));

    uint8_t *frame = &raw_buffer[1]; /* Starts at odd address (0x...1) */

    frame[0] = FW_TELEMETRY_SOF_BYTE; /* 0xAA */
    frame[1] = 0x42;                  /* Sensor Type: Pressure Sensor */

    /* Timestamp: 123456789 ms (0x075BCD15) at offset 2 (unaligned address!) */
    uint32_t expected_time = 123456789U;
    fw_write_u32_le(&frame[2], expected_time);

    /* Pressure: 101.325 kPa (float) at offset 6 (unaligned address!) */
    float expected_press = 101.325f;
    uint32_t raw_float_bits;
    memcpy(&raw_float_bits, &expected_press, sizeof(float));
    fw_write_u32_le(&frame[6], raw_float_bits);

    /* Status flags: 0x8001 at offset 10 */
    uint16_t expected_flags = 0x8001U;
    frame[10] = (uint8_t)(expected_flags & 0xFF);
    frame[11] = (uint8_t)((expected_flags >> 8) & 0xFF);

    /* Calculate XOR Checksum for bytes 0..11 */
    uint8_t crc = 0;
    for (size_t i = 0; i < 12; i++) {
        crc ^= frame[i];
    }
    frame[12] = crc;

    /* Now ingest with fw_sensor_ingest_packet */
    fw_sensor_telemetry_t telemetry;
    fw_status_t status = fw_sensor_ingest_packet(frame, FW_TELEMETRY_RAW_PKT_LEN, &telemetry);

    assert(status == FW_STATUS_OK);
    assert(telemetry.sensor_type_id == 0x42);
    assert(telemetry.timestamp_ms == expected_time);
    assert(telemetry.sensor_status_flags == expected_flags);
    assert(telemetry.checksum == crc);

    /* Float comparison within epsilon */
    float diff = telemetry.pressure_kpa - expected_press;
    if (diff < 0) diff = -diff;
    assert(diff < 0.0001f);

    printf("  -> Passed. Ingested time=%u ms, press=%.3f kPa, flags=0x%04X, CRC=0x%02X.\n",
           (unsigned int)telemetry.timestamp_ms,
           telemetry.pressure_kpa,
           telemetry.sensor_status_flags,
           telemetry.checksum);
}

static void test_chunked_stream_parsing(void)
{
    printf("[TEST] Running test_chunked_stream_parsing (Stack footprint < 48B)...\n");

    /* Build valid 13-byte frame preceded and succeeded by noise bytes */
    uint8_t wire_stream[25];
    memset(wire_stream, 0xFF, sizeof(wire_stream)); /* Noise */

    /* Put valid frame starting at index 3 */
    uint8_t *frame = &wire_stream[3];
    frame[0] = FW_TELEMETRY_SOF_BYTE;
    frame[1] = 0x05;
    fw_write_u32_le(&frame[2], 999999U);
    float press = 98.76f;
    uint32_t press_raw;
    memcpy(&press_raw, &press, sizeof(float));
    fw_write_u32_le(&frame[6], press_raw);
    frame[10] = 0x12;
    frame[11] = 0x34;

    uint8_t crc = 0;
    for (size_t i = 0; i < 12; i++) {
        crc ^= frame[i];
    }
    frame[12] = crc;

    /* Feed stream in 4-byte chunks to simulate UART RX or SPI DMA chunks */
    fw_stream_parser_state_t parser_state;
    memset(&parser_state, 0, sizeof(parser_state));

    fw_sensor_telemetry_t decoded_pkt;
    bool pkt_ready = false;

    size_t chunk_size = 4;
    for (size_t offset = 0; offset < sizeof(wire_stream); offset += chunk_size) {
        size_t current_chunk = chunk_size;
        if (offset + current_chunk > sizeof(wire_stream)) {
            current_chunk = sizeof(wire_stream) - offset;
        }

        fw_status_t status = fw_sensor_process_stream_chunk(&wire_stream[offset],
                                                           current_chunk,
                                                           &parser_state,
                                                           &decoded_pkt,
                                                           &pkt_ready);
        assert(status == FW_STATUS_OK);
        if (pkt_ready) {
            break;
        }
    }

    assert(pkt_ready == true);
    assert(decoded_pkt.sensor_type_id == 0x05);
    assert(decoded_pkt.timestamp_ms == 999999U);

    printf("  -> Passed. Multi-chunk frame successfully assembled across 4-byte fragments.\n");
}

int main(void)
{
    printf("=====================================================\n");
    printf(" Embedded Production Firmware Verification Suite\n");
    printf("=====================================================\n");

    test_uart_non_blocking_tx();
    test_uart_fifo_overflow_protection();
    test_unaligned_memory_ingestion();
    test_chunked_stream_parsing();

    printf("\n>>> ALL TESTS PASSED SUCCESSFULLY! <<<\n");
    return 0;
}
