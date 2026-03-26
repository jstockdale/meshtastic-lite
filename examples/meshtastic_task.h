/**
 * meshtastic_task.h — FreeRTOS integration for meshtastic-lite on T-Display-P4.
 *
 * Hardware notes (from t_display_p4_config.h):
 *   SPI1: SCLK=GPIO2, MOSI=GPIO3, MISO=GPIO4  (dedicated to SX1262, NOT shared with display)
 *   SX1262_CS   = GPIO 24
 *   SX1262_BUSY = GPIO 6
 *   SX1262_DIO1 = XL9535 IO17 (I2C GPIO expander — no direct HW interrupt)
 *   SX1262_RST  = XL9535 IO16 (assumed, also via expander)
 *
 * Because DIO1 is behind the XL9535 I2C expander, we cannot use hardware
 * interrupts. Instead, we poll the SX1262's IRQ status register via SPI
 * using RadioLib's getIrqFlags(). This is the same approach used by the
 * Meshtastic firmware's startReceiveDutyCycleAuto() path.
 *
 * Architecture:
 *   - meshTaskRx: FreeRTOS task that polls for incoming packets
 *   - meshTaskTx: optional task that drains a TX queue with CSMA/CA
 *   - Both tasks communicate with the main ADS-B app via FreeRTOS queues
 *
 * This is an EXAMPLE / SKELETON. Adapt to your RadioLib and XL9535 driver.
 *
 * Part of meshtastic-lite.
 */
#pragma once

// ── You would include your actual project headers here ──
// #include "driver/spi_master.h"
// #include "driver/gpio.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "RadioLib.h"      // jgromes/RadioLib
// #include "xl9535_driver.h" // your XL9535 I2C GPIO expander driver

#define MESH_CRYPTO_USE_MBEDTLS 1
#include "meshtastic.h"

// ─── T-Display-P4 Pin Definitions ──────────────────────────────────────────────

// SPI1 bus (dedicated to SX1262 — NOT shared with the MIPI display)
#define LORA_SPI_HOST    SPI2_HOST  // ESP-IDF SPI peripheral (SPI2 = HSPI on P4)
#define LORA_PIN_SCLK    2
#define LORA_PIN_MOSI    3
#define LORA_PIN_MISO    4
#define LORA_PIN_CS      24
#define LORA_PIN_BUSY    6

// DIO1 and RESET are on the XL9535 I2C GPIO expander.
// These aren't real GPIO numbers — they're expander pin IDs.
// Your XL9535 driver handles the actual I2C reads/writes.
#define LORA_XL_DIO1     17   // XL9535 IO17
#define LORA_XL_RST      16   // XL9535 IO16 (verify against your schematic)

// SPI bus clock — 4 MHz matches Meshtastic firmware default
#define LORA_SPI_FREQ_HZ 4000000

// ─── Message Types for Inter-task Communication ────────────────────────────────

/**
 * A decoded Meshtastic message passed to the main app via queue.
 */
struct MeshInboundMsg {
    uint32_t    from;
    uint32_t    to;
    uint32_t    id;
    MeshPortNum portnum;
    int8_t      channel_idx;    // -1 for PKI DM
    bool        is_pki;
    float       rssi;
    float       snr;
    uint8_t     payload[240];
    size_t      payload_len;
};

/**
 * An outbound message queued for TX.
 */
struct MeshOutboundMsg {
    uint32_t    to;             // MESH_ADDR_BROADCAST or specific node
    MeshPortNum portnum;
    int8_t      channel_idx;    // -1 = PKI DM to `to`
    bool        want_ack;
    uint8_t     payload[200];
    size_t      payload_len;
};

// ─── Global State ──────────────────────────────────────────────────────────────

// These would be declared in your .cpp file:
//
// MeshSession          g_mesh;
// QueueHandle_t        g_mesh_rx_queue;   // MeshInboundMsg
// QueueHandle_t        g_mesh_tx_queue;   // MeshOutboundMsg
// SX1262               g_lora;            // RadioLib instance
// SemaphoreHandle_t    g_lora_spi_mutex;  // if SPI bus is shared

// ─── Initialization ────────────────────────────────────────────────────────────

/**
 * Initialize the Meshtastic subsystem.
 * Call from app_main() AFTER the XL9535 I2C driver is initialized.
 *
 * Pseudocode — adapt to your specific RadioLib / ESP-IDF setup.
 */
static inline void meshSystemInit(/* your SPI handle, XL9535 handle, etc. */) {
    // ── 1. Initialize the session ──
    // MeshSession g_mesh;
    // uint32_t myNodeNum = getNodeNumFromMac(); // bottom 4 bytes of BLE MAC
    // g_mesh.init(REGION_ANZ, MODEM_LONG_FAST, ROLE_CLIENT, myNodeNum);  // -1 = hash freq slot
    // g_mesh.channels.addDefaultChannel();
    // g_mesh.pki.generate(); // or load from NVS

    // ── 2. Reset the SX1262 via XL9535 ──
    // xl9535_pin_write(LORA_XL_RST, 0);
    // vTaskDelay(pdMS_TO_TICKS(10));
    // xl9535_pin_write(LORA_XL_RST, 1);
    // vTaskDelay(pdMS_TO_TICKS(10));

    // ── 3. Configure RadioLib ──
    // MeshRadioConfig rc = g_mesh.radioConfig();
    //
    // SPI.begin(LORA_PIN_SCLK, LORA_PIN_MISO, LORA_PIN_MOSI, LORA_PIN_CS);
    //
    // // RadioLib SX1262 constructor: Module(CS, DIO1, RST, BUSY)
    // // DIO1 = RADIOLIB_NC (-1) because it's on the I2C expander
    // // RST  = RADIOLIB_NC (-1) because we manage it via XL9535
    // g_lora = new Module(LORA_PIN_CS, RADIOLIB_NC, RADIOLIB_NC, LORA_PIN_BUSY);
    //
    // int state = g_lora.begin(
    //     rc.frequency_mhz,
    //     rc.bandwidth_khz,
    //     rc.spreading_factor,
    //     rc.coding_rate,
    //     rc.sync_word,
    //     rc.tx_power_dbm,
    //     rc.preamble_length
    // );
    //
    // if (state != RADIOLIB_ERR_NONE) {
    //     ESP_LOGE("MESH", "SX1262 init failed: %d", state);
    //     return;
    // }
    //
    // // SX1262 uses DIO2 for RF switch (HPD16A module)
    // g_lora.setDio2AsRfSwitch(true);
    //
    // // Set CRC on (matches Meshtastic firmware)
    // g_lora.setCRC(RADIOLIB_SX126X_LORA_CRC_ON);
    //
    // // Start continuous receive
    // g_lora.startReceive();
    //
    // ESP_LOGI("MESH", "SX1262 ready: %.3f MHz, SF%d, BW%.0f",
    //          rc.frequency_mhz, rc.spreading_factor, rc.bandwidth_khz);

    // ── 4. Create queues ──
    // g_mesh_rx_queue = xQueueCreate(16, sizeof(MeshInboundMsg));
    // g_mesh_tx_queue = xQueueCreate(8, sizeof(MeshOutboundMsg));

    // ── 5. Start tasks ──
    // xTaskCreatePinnedToCore(meshRxTask, "mesh_rx", 8192, NULL, 5, NULL, 1);
    // xTaskCreatePinnedToCore(meshTxTask, "mesh_tx", 4096, NULL, 4, NULL, 1);
}

// ─── RX Task (polling) ─────────────────────────────────────────────────────────

/**
 * FreeRTOS task that polls the SX1262 for received packets.
 *
 * Since DIO1 is behind the XL9535 I2C expander, we can't use a GPIO ISR.
 * Instead we poll at ~50ms intervals. At SF11/BW250, a minimum packet is
 * ~230ms on-air, so 50ms polling gives us plenty of margin.
 *
 * Alternative: poll the XL9535 DIO1 pin via I2C, which avoids SPI traffic
 * when idle but adds I2C latency. Choose based on your bus utilization.
 */
static inline void meshRxTask(void *pvParameters) {
    (void)pvParameters;

    // uint8_t raw[256];
    // MeshRxResult result;
    // MeshInboundMsg msg;

    for (;;) {
        // ── Option A: Poll SX1262 IRQ flags directly via SPI ──
        // uint16_t irq = g_lora.getIrqFlags();
        // if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
        //     // Read the packet
        //     size_t len = 0;
        //     int state = g_lora.readData(raw, sizeof(raw));
        //     if (state == RADIOLIB_ERR_NONE) {
        //         len = g_lora.getPacketLength();
        //         float rssi = g_lora.getRSSI();
        //         float snr  = g_lora.getSNR();
        //
        //         // Process through meshtastic-lite
        //         if (g_mesh.processRx(raw, len, rssi, snr, &result)) {
        //             // Package for the main app
        //             msg.from          = result.packet.from;
        //             msg.to            = result.packet.to;
        //             msg.id            = result.packet.id;
        //             msg.portnum       = result.data.portnum;
        //             msg.channel_idx   = result.channel_idx;
        //             msg.is_pki        = result.is_pki;
        //             msg.rssi          = rssi;
        //             msg.snr           = snr;
        //             msg.payload_len   = result.data.payload_len;
        //             memcpy(msg.payload, result.data.payload,
        //                    result.data.payload_len);
        //
        //             xQueueSend(g_mesh_rx_queue, &msg, 0);
        //
        //             // If it's a position, you could also push ADS-B-like
        //             // tracks to your aircraft display...
        //         }
        //     }
        //
        //     // Restart receive
        //     g_lora.startReceive();
        // }
        //
        // ── Option B: Poll XL9535 DIO1 pin via I2C ──
        // bool dio1_high = xl9535_pin_read(LORA_XL_DIO1);
        // if (dio1_high) {
        //     // same readData() logic as above
        // }

        // vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ─── TX Task (CSMA/CA) ────────────────────────────────────────────────────────

/**
 * FreeRTOS task that drains the TX queue with CSMA/CA backoff.
 */
static inline void meshTxTask(void *pvParameters) {
    (void)pvParameters;

    // MeshOutboundMsg outMsg;
    // uint8_t frame[256];

    for (;;) {
        // Block until there's something to send
        // if (xQueueReceive(g_mesh_tx_queue, &outMsg, portMAX_DELAY) == pdTRUE) {
        //     size_t frame_len = 0;
        //
        //     if (outMsg.channel_idx >= 0) {
        //         // Channel broadcast/group message
        //         frame_len = g_mesh.buildTx(
        //             outMsg.channel_idx, outMsg.to, outMsg.portnum,
        //             outMsg.payload, outMsg.payload_len,
        //             outMsg.want_ack, true /*ok_to_mqtt*/, frame);
        //     } else {
        //         // PKI direct message
        //         frame_len = g_mesh.buildDmTx(
        //             outMsg.to, outMsg.portnum,
        //             outMsg.payload, outMsg.payload_len,
        //             outMsg.want_ack, frame,
        //             (uint32_t(*)())esp_random);
        //     }
        //
        //     if (frame_len > 0) {
        //         // CSMA/CA: wait for channel to be clear
        //         uint32_t delay = meshTxDelayMs(g_mesh.preset);
        //         vTaskDelay(pdMS_TO_TICKS(delay));
        //
        //         // Check channel activity (CAD)
        //         // The SX1262 can do CAD natively:
        //         // int cad = g_lora.scanChannel();
        //         // if (cad == RADIOLIB_LORA_DETECTED) {
        //         //     // Channel busy, re-queue with backoff
        //         //     vTaskDelay(pdMS_TO_TICKS(meshTxDelayMs(g_mesh.preset)));
        //         // }
        //
        //         int state = g_lora.transmit(frame, frame_len);
        //         if (state != RADIOLIB_ERR_NONE) {
        //             ESP_LOGW("MESH", "TX failed: %d", state);
        //         }
        //
        //         // Return to RX mode
        //         g_lora.startReceive();
        //     }
        // }
    }
}

// ─── Public API for Main App ───────────────────────────────────────────────────

/**
 * Send a text message on the default channel (broadcast).
 * Non-blocking — queues the message for the TX task.
 */
static inline bool meshSendText(const char *text) {
    // MeshOutboundMsg msg = {};
    // msg.to = MESH_ADDR_BROADCAST;
    // msg.portnum = PORT_TEXT_MESSAGE;
    // msg.channel_idx = 0;
    // msg.want_ack = false;
    // msg.payload_len = strlen(text);
    // if (msg.payload_len > sizeof(msg.payload)) msg.payload_len = sizeof(msg.payload);
    // memcpy(msg.payload, text, msg.payload_len);
    // return xQueueSend(g_mesh_tx_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE;
    (void)text;
    return false;
}

/**
 * Send a PKI-encrypted direct message to a specific node.
 */
static inline bool meshSendDm(uint32_t to_node, const char *text) {
    // MeshOutboundMsg msg = {};
    // msg.to = to_node;
    // msg.portnum = PORT_TEXT_MESSAGE;
    // msg.channel_idx = -1; // signals PKI DM
    // msg.want_ack = true;
    // msg.payload_len = strlen(text);
    // if (msg.payload_len > sizeof(msg.payload)) msg.payload_len = sizeof(msg.payload);
    // memcpy(msg.payload, text, msg.payload_len);
    // return xQueueSend(g_mesh_tx_queue, &msg, pdMS_TO_TICKS(100)) == pdTRUE;
    (void)to_node; (void)text;
    return false;
}

/**
 * Poll for received Meshtastic messages from your main loop / UI task.
 * Returns true if a message was available.
 */
static inline bool meshPollRx(MeshInboundMsg *msg) {
    // return xQueueReceive(g_mesh_rx_queue, msg, 0) == pdTRUE;
    (void)msg;
    return false;
}
