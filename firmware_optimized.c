/**
 * @file firmware_optimized.c
 * @brief Implementation of Production-Grade Embedded Firmware Module.
 *
 * Implements non-blocking UART ring buffering, atomic critical sections,
 * hardware fault-proof unaligned memory deserialization, and stream chunk parsing.
 */

#include "firmware_optimized.h"
#include <string.h>

/* ========================================================================== */
/* Private Static State & Buffers (BSS Allocation, Zero Stack Footprint)      */
/* ========================================================================== */

/**
 * @brief Static UART Transmit Ring Buffer allocated in BSS.
 * Keeps stack footprint at 0 bytes for FIFO storage.
 */
static fw_uart_fifo_t s_uart_tx_fifo;

/**
 * @brief Hardware Abstraction Layer operations pointer.
 */
static fw_uart_hal_t s_uart_hal;

/**
 * @brief Driver initialization flag.
 */
static bool s_driver_initialized = false;

/**
 * @brief State flag indicating whether the hardware transmitter is actively shifting.
 * Shared between background write calls and the TX ISR.
 */
static volatile bool s_tx_active = false;

/**
 * @brief Shared module statistics counters.
 */
static fw_telemetry_stats_t s_stats;

/* ========================================================================== */
/* Private Inline FIFO Helper Functions (Power-of-2 Optimized)                */
/* ========================================================================== */

/**
 * @brief Computes number of bytes currently stored in FIFO.
 *
 * Relies on unsigned integer wrap-around arithmetic:
 * Even when 'head' overflows 0xFFFF to 0x0000, ((uint16_t)(head - tail))
 * yields the exact element count without branching.
 */
static inline uint16_t fifo_get_count(const fw_uart_fifo_t *p_fifo)
{
    return (uint16_t)(p_fifo->head - p_fifo->tail);
}

/**
 * @brief Computes available free byte slots in FIFO.
 */
static inline uint16_t fifo_get_free(const fw_uart_fifo_t *p_fifo)
{
    return (uint16_t)(FW_UART_TX_FIFO_SIZE - fifo_get_count(p_fifo));
}

/**
 * @brief Checks if FIFO is empty.
 */
static inline bool fifo_is_empty(const fw_uart_fifo_t *p_fifo)
{
    return (p_fifo->head == p_fifo->tail);
}

/**
 * @brief Checks if FIFO is full.
 */
static inline bool fifo_is_full(const fw_uart_fifo_t *p_fifo)
{
    return (fifo_get_count(p_fifo) >= FW_UART_TX_FIFO_SIZE);
}

/**
 * @brief Pushes a single byte into the FIFO.
 * Uses bitwise AND mask instead of modulo operator (%) for single-cycle execution.
 */
static inline bool fifo_push(fw_uart_fifo_t *p_fifo, uint8_t byte)
{
    if (fifo_is_full(p_fifo)) {
        return false;
    }

    p_fifo->buffer[p_fifo->head & FW_UART_TX_FIFO_MASK] = byte;
    p_fifo->head++;
    return true;
}

/**
 * @brief Pops a single byte from the FIFO.
 */
static inline bool fifo_pop(fw_uart_fifo_t *p_fifo, uint8_t *p_byte)
{
    if (fifo_is_empty(p_fifo)) {
        return false;
    }

    *p_byte = p_fifo->buffer[p_fifo->tail & FW_UART_TX_FIFO_MASK];
    p_fifo->tail++;
    return true;
}

/* ========================================================================== */
/* UART Driver Implementation (Non-Blocking & Interrupt-Driven)              */
/* ========================================================================== */

fw_status_t fw_uart_init(const fw_uart_hal_t *p_hal)
{
    if ((p_hal == NULL) ||
        (p_hal->enable_tx_interrupt == NULL) ||
        (p_hal->disable_tx_interrupt == NULL) ||
        (p_hal->write_tx_data_reg == NULL)) {
        return FW_STATUS_ERR_NULL_PARAM;
    }

    fw_critical_state_t cs = fw_enter_critical();

    /* Copy HAL function pointers */
    s_uart_hal = *p_hal;

    /* Initialize Ring Buffer */
    s_uart_tx_fifo.head = 0U;
    s_uart_tx_fifo.tail = 0U;
    s_tx_active         = false;

    /* Reset transmitter interrupt on hardware */
    s_uart_hal.disable_tx_interrupt();

    s_driver_initialized = true;

    fw_exit_critical(cs);

    return FW_STATUS_OK;
}

fw_status_t fw_uart_write_async(const uint8_t *p_data, size_t len, size_t *p_bytes_saved)
{
    if ((p_data == NULL) || (len == 0U)) {
        return FW_STATUS_ERR_NULL_PARAM;
    }
    if (!s_driver_initialized) {
        return FW_STATUS_ERR_NULL_PARAM;
    }

    size_t bytes_pushed = 0U;

    /* Enter atomic critical section to safely inspect & manipulate FIFO and TX state */
    fw_critical_state_t cs = fw_enter_critical();

    /* Enqueue as many bytes as can fit in the ring buffer */
    while (bytes_pushed < len) {
        if (!fifo_push(&s_uart_tx_fifo, p_data[bytes_pushed])) {
            /* FIFO is full: record overflow drop metric */
            s_stats.tx_fifo_overflows++;
            break;
        }
        bytes_pushed++;
    }

    /* Update global stats atomically */
    s_stats.tx_bytes_enqueued += (uint32_t)bytes_pushed;

    /*
     * Transmission Kick-off Mechanism:
     * If transmitter hardware is currently idle (s_tx_active == false),
     * prime the hardware by popping the first byte and enabling TX interrupts.
     * Subsequent bytes will be fed seamlessly by fw_uart_tx_isr_handler().
     */
    if ((!s_tx_active) && (bytes_pushed > 0U)) {
        uint8_t first_byte = 0U;
        if (fifo_pop(&s_uart_tx_fifo, &first_byte)) {
            s_tx_active = true;
            s_stats.tx_bytes_transmitted++;
            s_uart_hal.write_tx_data_reg(first_byte);
            s_uart_hal.enable_tx_interrupt();
        }
    }

    fw_exit_critical(cs);

    if (p_bytes_saved != NULL) {
        *p_bytes_saved = bytes_pushed;
    }

    return (bytes_pushed == len) ? FW_STATUS_OK : FW_STATUS_ERR_FIFO_FULL;
}

fw_status_t fw_uart_putchar_async(uint8_t byte)
{
    size_t saved = 0U;
    return fw_uart_write_async(&byte, 1U, &saved);
}

void fw_uart_tx_isr_handler(void)
{
    /*
     * Context: Interrupt Service Routine (ISR).
     * Invoked when UART hardware Data Register is empty and ready for next byte.
     * No blocking or busy-waiting. Execution time is deterministic O(1).
     */
    uint8_t next_byte = 0U;

    if (fifo_pop(&s_uart_tx_fifo, &next_byte)) {
        /* Byte available: feed to UART transmitter */
        s_stats.tx_bytes_transmitted++;
        s_uart_hal.write_tx_data_reg(next_byte);
    } else {
        /*
         * FIFO is empty: transmission is complete.
         * Disable TX interrupt to prevent continuous spurious IRQs.
         */
        s_uart_hal.disable_tx_interrupt();
        s_tx_active = false;
    }
}

/* ========================================================================== */
/* Safe Memory Management & Telemetry Deserialization                         */
/* ========================================================================== */

fw_status_t fw_sensor_ingest_packet(const uint8_t *p_raw_frame,
                                    size_t length,
                                    fw_sensor_telemetry_t *p_telemetry)
{
    /* Defensive input validation */
    if ((p_raw_frame == NULL) || (p_telemetry == NULL)) {
        return FW_STATUS_ERR_NULL_PARAM;
    }
    if (length != FW_TELEMETRY_RAW_PKT_LEN) {
        return FW_STATUS_ERR_INVALID_SIZE;
    }

    /* 1. Verify Start of Frame (SOF) magic byte */
    if (p_raw_frame[0] != FW_TELEMETRY_SOF_BYTE) {
        return FW_STATUS_ERR_INVALID_SOF;
    }

    /* 2. Compute 8-bit XOR checksum over packet payload (bytes 0 to 11) */
    uint8_t calc_crc = 0U;
    for (size_t i = 0U; i < (FW_TELEMETRY_RAW_PKT_LEN - 1U); i++) {
        calc_crc ^= p_raw_frame[i];
    }

    uint8_t expected_crc = p_raw_frame[FW_TELEMETRY_RAW_PKT_LEN - 1U];
    if (calc_crc != expected_crc) {
        /* Atomically record CRC error metric */
        fw_critical_state_t cs = fw_enter_critical();
        s_stats.sensor_crc_errors++;
        fw_exit_critical(cs);
        return FW_STATUS_ERR_CRC_MISMATCH;
    }

    /*
     * 3. Safe unaligned field deserialization.
     *
     * Wire layout (Packed, 13 bytes total):
     * [0]       : SOF (0xAA)
     * [1]       : Sensor Type ID (uint8_t)
     * [2..5]    : Timestamp in ms (uint32_t, offset 2 -> MISALIGNED!)
     * [6..9]    : Pressure in kPa (float32, offset 6 -> MISALIGNED!)
     * [10..11]  : Status flags (uint16_t, offset 10)
     * [12]      : Checksum (uint8_t)
     *
     * Direct casting like `*(uint32_t*)&p_raw_frame[2]` would trigger a hardware
     * HardFault exception on Cortex-M0/M0+/M23/M33 and unaligned-trap RISC-V cores.
     * Using fw_read_*_le guarantees 100% portable, fault-free ingestion.
     */
    p_telemetry->sensor_type_id     = p_raw_frame[1];
    p_telemetry->timestamp_ms       = fw_read_u32_le(&p_raw_frame[2]);
    p_telemetry->pressure_kpa       = fw_read_float_le(&p_raw_frame[6]);
    p_telemetry->sensor_status_flags= fw_read_u16_le(&p_raw_frame[10]);
    p_telemetry->checksum           = expected_crc;

    /* Atomically record successfully ingested packet metric */
    fw_critical_state_t cs = fw_enter_critical();
    s_stats.sensor_packets_ingested++;
    fw_exit_critical(cs);

    return FW_STATUS_OK;
}

/* ========================================================================== */
/* Stack Optimization: Stream Processing Chunk Engine                         */
/* ========================================================================== */

fw_status_t fw_sensor_process_stream_chunk(const uint8_t *p_chunk,
                                           size_t chunk_len,
                                           fw_stream_parser_state_t *p_state,
                                           fw_sensor_telemetry_t *p_out_pkt,
                                           bool *p_pkt_ready)
{
    /*
     * Stack Frame Footprint Analysis:
     * - Parameter storage (in registers on ARM AAPCS/RISC-V): 5 pointers/scalars
     * - Local variables: 3 scalar bytes/integers ('i', 'byte', 'status')
     * - Total stack frame is strictly < 32 bytes (well below the 48-byte limit).
     * - No multi-kilobyte buffers allocated on stack!
     */
    if ((p_chunk == NULL) || (p_state == NULL) || (p_out_pkt == NULL) || (p_pkt_ready == NULL)) {
        return FW_STATUS_ERR_NULL_PARAM;
    }

    *p_pkt_ready = false;

    for (size_t i = 0U; i < chunk_len; i++) {
        uint8_t byte = p_chunk[i];

        if (!p_state->sync_found) {
            /* Hunt for Start of Frame marker */
            if (byte == FW_TELEMETRY_SOF_BYTE) {
                p_state->sync_found = true;
                p_state->rx_chunk_buf[0] = byte;
                p_state->rx_index = 1U;
            }
        } else {
            /* Assemble frame into caller-provided state buffer */
            p_state->rx_chunk_buf[p_state->rx_index] = byte;
            p_state->rx_index++;

            if (p_state->rx_index >= FW_TELEMETRY_RAW_PKT_LEN) {
                /* Full frame received: ingest & decode */
                fw_status_t status = fw_sensor_ingest_packet(p_state->rx_chunk_buf,
                                                             FW_TELEMETRY_RAW_PKT_LEN,
                                                             p_out_pkt);
                /* Reset sync state for subsequent frame */
                p_state->sync_found = false;
                p_state->rx_index   = 0U;

                if (status == FW_STATUS_OK) {
                    *p_pkt_ready = true;
                    return FW_STATUS_OK;
                }
            }
        }
    }

    return FW_STATUS_OK;
}

/* ========================================================================== */
/* Atomic Statistics Snapshot & Reset                                         */
/* ========================================================================== */

fw_status_t fw_stats_get_snapshot(fw_telemetry_stats_t *p_dest)
{
    if (p_dest == NULL) {
        return FW_STATUS_ERR_NULL_PARAM;
    }

    /*
     * Take an atomic snapshot under critical section to guarantee
     * multi-word coherence (preventing reading intermediate states).
     */
    fw_critical_state_t cs = fw_enter_critical();
    p_dest->tx_bytes_enqueued       = s_stats.tx_bytes_enqueued;
    p_dest->tx_bytes_transmitted    = s_stats.tx_bytes_transmitted;
    p_dest->tx_fifo_overflows       = s_stats.tx_fifo_overflows;
    p_dest->sensor_packets_ingested = s_stats.sensor_packets_ingested;
    p_dest->sensor_crc_errors       = s_stats.sensor_crc_errors;
    fw_exit_critical(cs);

    return FW_STATUS_OK;
}

void fw_stats_reset(void)
{
    fw_critical_state_t cs = fw_enter_critical();
    s_stats.tx_bytes_enqueued       = 0U;
    s_stats.tx_bytes_transmitted    = 0U;
    s_stats.tx_fifo_overflows       = 0U;
    s_stats.sensor_packets_ingested = 0U;
    s_stats.sensor_crc_errors       = 0U;
    fw_exit_critical(cs);
}
