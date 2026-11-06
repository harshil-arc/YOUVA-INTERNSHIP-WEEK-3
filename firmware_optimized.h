/**
 * @file firmware_optimized.h
 * @brief Production-Grade Embedded Firmware Module for 32-bit MCUs (ARM Cortex-M / RISC-V).
 *
 * This module implements:
 * 1. An asynchronous, non-blocking UART Transmit Ring Buffer (power-of-2 circular FIFO).
 * 2. Unaligned memory access protection and safe telemetry packet deserialization.
 * 3. Re-entrant, nested-safe atomic critical section primitives with compiler barriers.
 * 4. Ultra-low stack footprint streaming architecture (< 48 bytes per frame).
 *
 * Coding Standards: MISRA C:2012 principles, strict typing, zero busy-wait loops.
 */

#ifndef FIRMWARE_OPTIMIZED_H
#define FIRMWARE_OPTIMIZED_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* Standard Includes                                                          */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ========================================================================== */
/* Configuration & Constants                                                  */
/* ========================================================================== */

/**
 * @brief Size of the UART Transmit Ring Buffer in bytes.
 * @note MUST be an exact power of 2 (e.g., 64, 128, 256) for O(1) bitwise masking.
 */
#define FW_UART_TX_FIFO_SIZE        (128U)
#define FW_UART_TX_FIFO_MASK        (FW_UART_TX_FIFO_SIZE - 1U)

/* Compile-time verification that FW_UART_TX_FIFO_SIZE is a non-zero power of 2 */
#if ((FW_UART_TX_FIFO_SIZE == 0U) || ((FW_UART_TX_FIFO_SIZE & FW_UART_TX_FIFO_MASK) != 0U))
    #error "FW_UART_TX_FIFO_SIZE must be an exact power of 2 (e.g., 64, 128, 256)!"
#endif

/**
 * @brief Telemetry protocol framing constants.
 */
#define FW_TELEMETRY_SOF_BYTE       (0xAAU) /**< Start of Frame delimiter */
#define FW_TELEMETRY_RAW_PKT_LEN    (13U)   /**< Exact wire length of packed sensor frame */

/* ========================================================================== */
/* Status and Error Codes                                                     */
/* ========================================================================== */

/**
 * @brief Module-wide return status enumeration.
 */
typedef enum {
    FW_STATUS_OK                    = 0,  /**< Operation successful */
    FW_STATUS_ERR_NULL_PARAM        = 1,  /**< Null pointer passed to function */
    FW_STATUS_ERR_FIFO_FULL         = 2,  /**< Ring buffer has insufficient capacity */
    FW_STATUS_ERR_FIFO_EMPTY        = 3,  /**< Ring buffer has no data available */
    FW_STATUS_ERR_INVALID_SIZE      = 4,  /**< Size or length parameter out of bounds */
    FW_STATUS_ERR_INVALID_SOF       = 5,  /**< Missing or invalid Start of Frame byte */
    FW_STATUS_ERR_CRC_MISMATCH      = 6   /**< Computed frame CRC does not match wire CRC */
} fw_status_t;

/* ========================================================================== */
/* Data Structures                                                            */
/* ========================================================================== */

/**
 * @brief Circular FIFO Ring Buffer structure for asynchronous serial TX.
 * 
 * Uses free-running, monotonically increasing 16-bit indices.
 * - 'head' is modified solely by the producer (application thread).
 * - 'tail' is modified solely by the consumer (UART TX ISR).
 * - Array indexing is computed via bitwise masking: (index & FW_UART_TX_FIFO_MASK).
 */
typedef struct {
    uint8_t buffer[FW_UART_TX_FIFO_SIZE];   /**< Storage array */
    volatile uint16_t head;                 /**< Free-running write index (Producer) */
    volatile uint16_t tail;                 /**< Free-running read index (Consumer) */
} fw_uart_fifo_t;

/**
 * @brief Hardware Abstraction Layer (HAL) hooks for UART peripheral control.
 * 
 * Allows complete decoupling from vendor-specific registers (e.g. STM32, NXP, TI, ESP32).
 */
typedef struct {
    void (*enable_tx_interrupt)(void);      /**< Function to enable TX Ready/Empty IRQ */
    void (*disable_tx_interrupt)(void);     /**< Function to disable TX Ready/Empty IRQ */
    void (*write_tx_data_reg)(uint8_t byte);/**< Function to write single byte to UART DR/TDR */
} fw_uart_hal_t;

/**
 * @brief Global statistics counters shared between Application and ISRs.
 * 
 * Marked volatile and updated exclusively inside atomic critical sections.
 */
typedef struct {
    volatile uint32_t tx_bytes_enqueued;    /**< Total bytes written by background task */
    volatile uint32_t tx_bytes_transmitted; /**< Total bytes emitted by UART TX ISR */
    volatile uint32_t tx_fifo_overflows;    /**< Drop count due to full ring buffer */
    volatile uint32_t sensor_packets_ingested; /**< Total valid sensor packets parsed */
    volatile uint32_t sensor_crc_errors;    /**< Total sensor packets dropped on CRC */
} fw_telemetry_stats_t;

/**
 * @brief Properly aligned, native C structure for parsed sensor telemetry.
 * 
 * Ensures natural 32-bit and 16-bit alignment on all architectures,
 * completely eliminating unaligned pointer dereference exceptions.
 */
typedef struct {
    uint32_t timestamp_ms;                  /**< 32-bit system timestamp (ms) */
    float    pressure_kpa;                  /**< 32-bit single-precision float reading */
    uint16_t sensor_status_flags;           /**< 16-bit hardware/sensor status bitfield */
    uint8_t  sensor_type_id;                /**< 8-bit sensor classification ID */
    uint8_t  checksum;                      /**< 8-bit checksum value */
} fw_sensor_telemetry_t;

/**
 * @brief Stream processing chunk state machine for ultra-low stack ingestion.
 */
typedef struct {
    uint8_t  rx_chunk_buf[FW_TELEMETRY_RAW_PKT_LEN]; /**< Holding buffer for framing */
    uint8_t  rx_index;                               /**< Current byte position in frame */
    bool     sync_found;                             /**< Frame synchronizer lock flag */
} fw_stream_parser_state_t;

/* ========================================================================== */
/* Critical Section & Atomic Primitives                                       */
/* ========================================================================== */

/**
 * @brief Architectural Interrupt State type for re-entrant critical sections.
 */
typedef uint32_t fw_critical_state_t;

/**
 * @brief Enters an atomic critical section by disabling global interrupts and
 *        returning the previous interrupt state register.
 *
 * Implements re-entrant nesting: if interrupts were already masked, the returned
 * state preserves that mask so an inner exit does not prematurely enable them.
 * Includes a full compiler memory barrier to prevent instruction reordering.
 *
 * @return Saved interrupt status register (e.g. PRIMASK for ARM, MSTATUS for RISC-V).
 */
static inline fw_critical_state_t fw_enter_critical(void)
{
    fw_critical_state_t state = 0U;

#if defined(__arm__) || defined(__thumb__)
    /* ARM Cortex-M Architecture: read PRIMASK then disable interrupts (cpsid i) */
    __asm volatile (
        "mrs %0, primask\n\t"
        "cpsid i"
        : "=r" (state)
        :
        : "memory"
    );
#elif defined(__riscv)
    /* RISC-V Architecture: clear MIE (Machine Interrupt Enable) bit in mstatus */
    __asm volatile (
        "csrrci %0, mstatus, 8"
        : "=r" (state)
        :
        : "memory"
    );
#else
    /* Host / Unit-Testing fallback: simulation marker with compiler barrier */
    __asm volatile ("" ::: "memory");
    (void)state;
#endif

    return state;
}

/**
 * @brief Exits an atomic critical section by restoring the previous interrupt state.
 *
 * Restores interrupt configuration back to its pre-critical-section condition.
 * Includes a full compiler memory barrier to guarantee all shared memory writes
 * are retired before interrupts are re-enabled.
 *
 * @param[in] state Saved interrupt status register returned by fw_enter_critical().
 */
static inline void fw_exit_critical(fw_critical_state_t state)
{
#if defined(__arm__) || defined(__thumb__)
    /* ARM Cortex-M Architecture: restore PRIMASK */
    __asm volatile (
        "msr primask, %0"
        :
        : "r" (state)
        : "memory"
    );
#elif defined(__riscv)
    /* RISC-V Architecture: restore mstatus */
    __asm volatile (
        "csrw mstatus, %0"
        :
        : "r" (state)
        : "memory"
    );
#else
    /* Host / Unit-Testing fallback */
    (void)state;
    __asm volatile ("" ::: "memory");
#endif
}

/* ========================================================================== */
/* Memory Alignment & Unaligned Access Helpers                                */
/* ========================================================================== */

/**
 * @brief Safely reads a 16-bit unsigned integer from an unaligned buffer (Little-Endian).
 * @param[in] p_src Pointer to raw byte stream (can have any byte alignment).
 * @return 16-bit unsigned value assembled safely via byte shifting.
 */
static inline uint16_t fw_read_u16_le(const uint8_t *p_src)
{
    /* Byte-wise extraction prevents unaligned LDRH hardware faults */
    return (uint16_t)(((uint16_t)p_src[0]) |
                     (((uint16_t)p_src[1]) << 8U));
}

/**
 * @brief Safely reads a 32-bit unsigned integer from an unaligned buffer (Little-Endian).
 * @param[in] p_src Pointer to raw byte stream (can have any byte alignment).
 * @return 32-bit unsigned value assembled safely via byte shifting.
 */
static inline uint32_t fw_read_u32_le(const uint8_t *p_src)
{
    /* Byte-wise extraction prevents unaligned LDR/LDRD hardware faults */
    return ((uint32_t)p_src[0])        |
          (((uint32_t)p_src[1]) << 8U)  |
          (((uint32_t)p_src[2]) << 16U) |
          (((uint32_t)p_src[3]) << 24U);
}

/**
 * @brief Safely reads a 32-bit single-precision float from an unaligned buffer (IEEE-754 LE).
 * @param[in] p_src Pointer to raw byte stream (can have any byte alignment).
 * @return 32-bit float value copied safely into a naturally aligned float variable.
 */
static inline float fw_read_float_le(const uint8_t *p_src)
{
    float val;
    uint32_t raw = fw_read_u32_le(p_src);
    /* Compiler optimizes memcpy into register move without stack overhead */
    __builtin_memcpy(&val, &raw, sizeof(float));
    return val;
}

/**
 * @brief Safely writes a 32-bit unsigned integer into an unaligned buffer (Little-Endian).
 * @param[out] p_dst Destination buffer pointer (can have any byte alignment).
 * @param[in]  val   32-bit value to store.
 */
static inline void fw_write_u32_le(uint8_t *p_dst, uint32_t val)
{
    p_dst[0] = (uint8_t)(val & 0xFFU);
    p_dst[1] = (uint8_t)((val >> 8U)  & 0xFFU);
    p_dst[2] = (uint8_t)((val >> 16U) & 0xFFU);
    p_dst[3] = (uint8_t)((val >> 24U) & 0xFFU);
}

/* ========================================================================== */
/* Public API Function Declarations                                           */
/* ========================================================================== */

/**
 * @brief Initializes the UART asynchronous transmit driver and FIFO.
 * @param[in] p_hal Pointer to HAL operations structure.
 * @return FW_STATUS_OK on success, or error code on invalid parameter.
 */
fw_status_t fw_uart_init(const fw_uart_hal_t *p_hal);

/**
 * @brief Non-blocking write to UART transmit ring buffer.
 *
 * Enqueues bytes into the circular FIFO without blocking. If the transmitter is
 * currently idle, kicks off transmission and enables the TX hardware interrupt.
 *
 * @param[in]  p_data        Pointer to source data buffer.
 * @param[in]  len           Number of bytes to transmit.
 * @param[out] p_bytes_saved Pointer to store count of successfully enqueued bytes.
 * @return FW_STATUS_OK on success, FW_STATUS_ERR_FIFO_FULL if buffer has insufficient space.
 */
fw_status_t fw_uart_write_async(const uint8_t *p_data, size_t len, size_t *p_bytes_saved);

/**
 * @brief Non-blocking single byte write helper.
 * @param[in] byte Character to enqueue.
 * @return FW_STATUS_OK on success, FW_STATUS_ERR_FIFO_FULL if buffer is full.
 */
fw_status_t fw_uart_putchar_async(uint8_t byte);

/**
 * @brief UART Transmit Interrupt Service Routine (ISR) handler.
 *
 * MUST be called directly from the MCU's hardware UART TX interrupt handler.
 * Runs in interrupt context: pops the next byte from the ring buffer into the UART DR.
 * Automatically disables TX interrupts when the ring buffer is exhausted.
 */
void fw_uart_tx_isr_handler(void);

/**
 * @brief Safe ingestion of raw sensor telemetry frame with zero unaligned accesses.
 *
 * Verifies frame start delimiter, calculates XOR checksum across the frame, and
 * extracts multi-byte fields (timestamp, float pressure, status flags) using
 * safe alignment-agnostic primitives.
 *
 * @param[in]  p_raw_frame   Pointer to unaligned raw byte stream.
 * @param[in]  length        Length of the incoming frame in bytes.
 * @param[out] p_telemetry   Pointer to naturally aligned telemetry destination struct.
 * @return FW_STATUS_OK on success, or specific error code on failure.
 */
fw_status_t fw_sensor_ingest_packet(const uint8_t *p_raw_frame,
                                    size_t length,
                                    fw_sensor_telemetry_t *p_telemetry);

/**
 * @brief Ingests arbitrary-length data streams in small chunks with minimal stack usage.
 *
 * Processes streams chunk-by-chunk using a persistent parser state object, ensuring
 * that the function's stack frame remains strictly below 48 bytes.
 *
 * @param[in]     p_chunk       Pointer to incoming byte chunk.
 * @param[in]     chunk_len     Length of chunk.
 * @param[in,out] p_state       Persistent stream parser state.
 * @param[out]    p_out_pkt     Destination for parsed packet once a full frame is decoded.
 * @param[out]    p_pkt_ready   Set to true if a complete packet was parsed, false otherwise.
 * @return FW_STATUS_OK on success, or error code on invalid parameter.
 */
fw_status_t fw_sensor_process_stream_chunk(const uint8_t *p_chunk,
                                           size_t chunk_len,
                                           fw_stream_parser_state_t *p_state,
                                           fw_sensor_telemetry_t *p_out_pkt,
                                           bool *p_pkt_ready);

/**
 * @brief Atomically snapshots the shared telemetry statistics counters.
 * @param[out] p_dest Destination structure to receive the snapshot.
 * @return FW_STATUS_OK on success, FW_STATUS_ERR_NULL_PARAM if pointer is NULL.
 */
fw_status_t fw_stats_get_snapshot(fw_telemetry_stats_t *p_dest);

/**
 * @brief Atomically resets all shared telemetry statistics counters.
 */
void fw_stats_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* FIRMWARE_OPTIMIZED_H */
