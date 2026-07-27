/**
 * @file    SubGHzBridge_M5_v1_2_baseline.ino
 * @brief   Bridge IP sub-GHz sobre GFSK 250 kbps — Elecrow ThinkNode M5
 *
 * ══════════════════════════════════════════════════════════════════════════
 *  v1.2 — BASELINE FUNCIONAL
 * ══════════════════════════════════════════════════════════════════════════
 *  BASADO EN v1.1 con los siguientes fixes:
 *
 *  FIX 1 — rx_last_ok inicializado en 0 (no 0xFF) + flag rx_synced
 *  FIX 2 — ACK solo se procesa si tx_cur != NULL
 *  FIX 3 — Recuperacion de desync en secuencia (FIRST bit)
 *  FIX 4 — tx_retries se resetea al recibir DATA valido del peer
 *  FIX 5 — Modo BENCHMARK interno
 *  FIX 6 — Log serial con mas detalle
 *  FIX 7 — Comandos seriales (start/stop/bm/stats/reset)
 *
 *  v1.2: NAPT corregido — Gateway NAPT en SG, Cliente NAPT en AP
 *  VERIFICADO: ping WAN 8.8.8.8 → 256ms RTT, Telegram conectado, ~22 kbps
 *
 * ══════════════════════════════════════════════════════════════════════════
 */

#define ROLE_GATEWAY   1      // 1 = GATEWAY (STA+MASTER) | 0 = CLIENTE (AP+SLAVE)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RadioLib.h>
#include <GxEPD2_BW.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <esp_netif.h>
#include <esp_mac.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <lwip/netif.h>
#include <lwip/ip4_frag.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

// ============================================================
// 1. RADIO
// ============================================================
#define RF_FREQ_MHZ      915.0f
#define RF_BITRATE_KBPS  250.0f
#define RF_FDEV_KHZ       80.0f
#define RF_RXBW_KHZ      467.0f
#define RF_PREAMBLE_BITS    32
#define RF_TCXO_V          3.3f
#define RF_TCXO_US         500
#define RF_POWER_DBM        20      // 14 banco / 22 alcance / -9 minimo
#define RF_CURRENT_LIMIT   140.0f
#define RF_SPI_HZ      10000000

static uint8_t RF_SYNC[4] = { 0x9C, 0x5A, 0x3B, 0x67 };

// ============================================================
// 2. MAC / FRAGMENTACION
// ============================================================
#define SG_HDR_LEN         6
#define SG_FRAG_PAYLOAD  240        // 246 B total -> ~8.22 ms @250kbps
#define SG_MAX_ETH      1514        // MTU 1500 + Ethernet header
#define SG_REASM_BUF    1600

#define SG_ACK_WAIT_MS       60     // 30→60ms para mejor PER
#define SG_GUARD_MS       60
#define SG_MAX_RETRIES     3

#define SG_TXQ_DEPTH      12
#define SG_TXQ_MAX_BYTES 3000
#define SG_RXQ_DEPTH      12

#define SELFTEST_FRAMES  200

#define SG_T_DATA  0x00
#define SG_T_NULL  0x40

#define SG_F_FIRST 0x20
#define SG_F_MF    0x10

#define O_CTRL  0
#define O_SEQ   1
#define O_ACK   2
#define O_RSSI  3
#define O_NOISE 4
#define O_RSVD  5

// ============================================================
// 3. RED
// ============================================================
#if ROLE_GATEWAY
  #define WIFI_STA_SSID  "YourSSID"
  #define WIFI_STA_PASS  "YoutPASS"
  #define SG_IP    "10.0.0.1"
  #define SG_GW    "0.0.0.0"
  #define SG_MASK  "255.255.255.252"
#else
  #define WIFI_AP_SSID   "SubGHz-Bridge"
  #define WIFI_AP_PASS   "bridge1234"
  #define WIFI_AP_CHAN   6
  #define SG_IP    "10.0.0.2"
  #define SG_GW    "10.0.0.1"
  #define SG_MASK  "255.255.255.252"
#endif

// ============================================================
// 4. PINOUT M5
// ============================================================
#define PIN_I2C_SDA  48
#define PIN_I2C_SCL  47
#define PCA9557_ADDR 0x18
#define PCA_REG_OUT  0x01
#define PCA_REG_CFG  0x03
#define PCA_LED_ENABLE 2
#define PCA_POWER_EN   4
#define PCA_EINK_EN    5

#define PIN_LORA_SCK  16
#define PIN_LORA_MISO  7
#define PIN_LORA_MOSI 15
#define PIN_LORA_CS   17
#define PIN_LORA_RST   6
#define PIN_LORA_BUSY  5
#define PIN_LORA_DIO1  4
#define PIN_LORA_PWREN 46

#define PIN_EPD_SCK  38
#define PIN_EPD_MOSI 45
#define PIN_EPD_CS   39
#define PIN_EPD_DC   40
#define PIN_EPD_RST  41
#define PIN_EPD_BUSY 42

#define PIN_BTN1     21
#define PIN_BTN2     14
#define PIN_BUZZER    9
#define PIN_BAT_ADC   8
#define ADC_MULT   2.11f
#define DEBOUNCE_MS   60UL
#define LONGPRESS_MS 800UL

// ============================================================
// 5. OBJETOS
// ============================================================
SPIClass spi_lora(FSPI);
SPIClass spi_epd (HSPI);
SX1262 radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1,
                          PIN_LORA_RST, PIN_LORA_BUSY, spi_lora,
                          SPISettings(RF_SPI_HZ, MSBFIRST, SPI_MODE0));
GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(
    GxEPD2_154_D67(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY));

// ============================================================
// 6. ESTADO
// ============================================================
typedef struct { uint16_t len; uint8_t data[]; } sg_frame_t;

struct Stats {
    uint32_t frag_tx, frag_rx, frag_lost, crc_err, retx;
    uint32_t eth_tx, eth_rx, eth_drop_q, eth_drop_arq, reasm_err;
    uint32_t bytes_down, bytes_up, t0_ms;
    float    rssi_loc, noise_loc;
    int8_t   rssi_rem;
    float    per, kbps_down, kbps_up, cycle_ms;
    bool     up;
    uint32_t t_last_rx;
};
static volatile Stats st = {0};
static volatile bool  running = false, audible = false, do_selftest = false;
static volatile bool  benchmark = false;            // FIX 5: modo benchmark
static volatile uint32_t txq_bytes = 0;

static uint64_t cyc_sum = 0;  static uint32_t cyc_n = 0;
static float tcxo_excess_ms = -1.0f, toa_theo_ms = 0;

static TaskHandle_t h_mac = NULL;
static QueueHandle_t q_tx = NULL, q_rx = NULL;
static esp_netif_t  *sg_netif = NULL;

static uint8_t pca_out = 0, bat_pct = 0;
static char    net_ssid[24] = "", net_pass[24] = "", net_ip[20] = "---";

static uint8_t tx_pkt[SG_HDR_LEN + SG_FRAG_PAYLOAD];
static uint8_t rx_pkt[SG_HDR_LEN + SG_FRAG_PAYLOAD];
static uint8_t reasm[SG_REASM_BUF];
static uint16_t reasm_len = 0;
static bool     reasm_active = false;

static sg_frame_t *tx_cur = NULL;
static uint16_t    tx_off = 0;
static uint8_t     tx_seq = 0;
static uint8_t     tx_retries = 0;
static uint8_t     rx_expect = 0;
static uint8_t     rx_last_ok = 0;         // FIX 1: 0 en vez de 0xFF
static bool        rx_synced = false;       // FIX 1: flag de primera recepcion

static inline uint32_t us_now() { return (uint32_t)esp_timer_get_time(); }

// ============================================================
// 7. ISR + SINCRONIZACION
// ============================================================
void IRAM_ATTR isr_dio1() {
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(h_mac, &hpw);
    if (hpw) portYIELD_FROM_ISR();
}
static inline bool wait_dio1(uint32_t ms) {
    return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms)) > 0;
}
static inline void notify_flush() { ulTaskNotifyTake(pdTRUE, 0); }

// ============================================================
// 8. PCA9557 / BUZZER / BATERIA
// ============================================================
static bool pca_init() {
    Wire.beginTransmission(PCA9557_ADDR);
    Wire.write(PCA_REG_CFG); Wire.write(0xC1);
    if (Wire.endTransmission()) return false;
    pca_out = 0;
    Wire.beginTransmission(PCA9557_ADDR);
    Wire.write(PCA_REG_OUT); Wire.write(pca_out);
    return Wire.endTransmission() == 0;
}
static void pca_w(uint8_t p, bool v) {
    if (v) pca_out |= (1u << p); else pca_out &= ~(1u << p);
    Wire.beginTransmission(PCA9557_ADDR);
    Wire.write(PCA_REG_OUT); Wire.write(pca_out);
    Wire.endTransmission();
}
static void beep(uint16_t hz, uint16_t ms) {
    ledcWriteTone(PIN_BUZZER, hz); delay(ms); ledcWriteTone(PIN_BUZZER, 0);
}
static uint8_t bat_read() {
    uint32_t s = 0;
    for (int i = 0; i < 16; i++) { s += analogRead(PIN_BAT_ADC); delay(1); }
    float v = (s / 16 / 4095.0f) * 3.1f * ADC_MULT;
    return (uint8_t)constrain((v - 3.0f) / 1.2f * 100.0f, 0, 100);
}

// ============================================================
// 9. RADIO
// ============================================================
static float fsk_rssi() {
    uint32_t t0 = millis();
    while (digitalRead(PIN_LORA_BUSY) && (millis() - t0 < 10)) { }
    uint8_t s[3];
    spi_lora.beginTransaction(SPISettings(RF_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_LORA_CS, LOW);
    spi_lora.transfer(0x14);
    spi_lora.transfer(0x00);
    s[0] = spi_lora.transfer(0x00);
    s[1] = spi_lora.transfer(0x00);
    s[2] = spi_lora.transfer(0x00);
    digitalWrite(PIN_LORA_CS, HIGH);
    spi_lora.endTransaction();
    return -(float)s[1] / 2.0f;
}

static bool radio_init() {
    digitalWrite(PIN_LORA_PWREN, HIGH); delay(20);
    int s = radio.beginFSK(RF_FREQ_MHZ, RF_BITRATE_KBPS, RF_FDEV_KHZ,
                           RF_RXBW_KHZ, RF_POWER_DBM, RF_PREAMBLE_BITS,
                           RF_TCXO_V, false);
    if (s != RADIOLIB_ERR_NONE) { Serial.printf("[FATAL] beginFSK=%d\n", s); return false; }
    radio.setTCXO(RF_TCXO_V, RF_TCXO_US);
    radio.setDio2AsRfSwitch(true);
    radio.setDataShaping(RADIOLIB_SHAPING_0_5);
    radio.setSyncWord(RF_SYNC, sizeof(RF_SYNC));
    radio.setCRC(2);
    radio.setWhitening(true);
    radio.setCurrentLimit(RF_CURRENT_LIMIT);
    radio.setRxBoostedGainMode(true);
    radio.setDio1Action(isr_dio1);
    toa_theo_ms = (float)radio.getTimeOnAir(SG_HDR_LEN + SG_FRAG_PAYLOAD) / 1000.0f;
    Serial.printf("[RF] GFSK 915/250k | SPI %d MHz | TCXO %u us | ToA %.2f ms\n",
                  RF_SPI_HZ / 1000000, RF_TCXO_US, toa_theo_ms);
    return true;
}

static float measure_noise(uint16_t n = 96) {
    notify_flush(); radio.startReceive(); delay(5);
    float a = 0; for (uint16_t i = 0; i < n; i++) { a += radio.getRSSI(false); delay(1); }
    radio.standby(); return a / n;
}

static void run_selftest() {
    uint64_t acc = 0; uint32_t n = 0;
    uint8_t buf[SG_HDR_LEN + SG_FRAG_PAYLOAD];
    for (uint32_t i = 0; i < SELFTEST_FRAMES; i++) {
        memset(buf, (uint8_t)i, sizeof(buf));
        buf[O_CTRL] = SG_T_NULL;
        notify_flush();
        uint32_t t0 = us_now();
        radio.startTransmit(buf, sizeof(buf));
        if (!wait_dio1(SG_GUARD_MS)) { radio.finishTransmit(); continue; }
        uint32_t t1 = us_now();
        radio.finishTransmit();
        acc += (t1 - t0); n++;
    }
    if (!n) { tcxo_excess_ms = -1.0f; return; }
    tcxo_excess_ms = (float)acc / n / 1000.0f - toa_theo_ms;
    Serial.printf("[TEST] exceso sobre ToA = %.2f ms\n", tcxo_excess_ms);
    beep(tcxo_excess_ms < 2.0f ? 1800 : 400, 120);
}

// ============================================================
// 10. NETIF PERSONALIZADO
// ============================================================
typedef struct { esp_netif_driver_base_t base; } sg_driver_t;
static sg_driver_t sg_drv;

static esp_err_t sg_transmit(void *h, void *buffer, size_t len) {
    if (!running || len == 0 || len > SG_MAX_ETH) return ESP_FAIL;

    while (txq_bytes + len > SG_TXQ_MAX_BYTES) {
        sg_frame_t *old = NULL;
        if (xQueueReceive(q_tx, &old, 0) != pdTRUE) break;
        txq_bytes -= old->len; free(old); st.eth_drop_q++;
    }

    sg_frame_t *f = (sg_frame_t*)malloc(sizeof(sg_frame_t) + len);
    if (!f) { st.eth_drop_q++; return ESP_ERR_NO_MEM; }
    f->len = len; memcpy(f->data, buffer, len);
    if (xQueueSend(q_tx, &f, 0) != pdTRUE) { free(f); st.eth_drop_q++; return ESP_FAIL; }
    txq_bytes += len;
    return ESP_OK;
}

static void sg_free_rx(void *h, void *buffer) { if (buffer) free(buffer); }

static esp_err_t sg_post_attach(esp_netif_t *netif, void *args) {
    sg_driver_t *drv = (sg_driver_t*)args;
    drv->base.netif = netif;
    const esp_netif_driver_ifconfig_t ifcfg = {
        .handle = drv,
        .transmit = sg_transmit,
        .transmit_wrap = NULL,
        .driver_free_rx_buffer = sg_free_rx
    };
    return esp_netif_set_driver_config(netif, &ifcfg);
}

static bool sg_netif_start() {
    esp_netif_ip_info_t ip = {};
    ip.ip.addr      = esp_ip4addr_aton(SG_IP);
    ip.gw.addr      = esp_ip4addr_aton(SG_GW);
    ip.netmask.addr = esp_ip4addr_aton(SG_MASK);

    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_ETH();
    base.if_key   = "SG";
    base.if_desc  = "subghz";
    base.ip_info  = &ip;
    base.flags    = (esp_netif_flags_t)(ESP_NETIF_FLAG_AUTOUP);
#if ROLE_GATEWAY
    base.route_prio = 10;
#else
    base.route_prio = 100;
#endif

    esp_netif_config_t cfg = { .base = &base, .driver = NULL,
                               .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH };
    sg_netif = esp_netif_new(&cfg);
    if (!sg_netif) { Serial.println("[FATAL] esp_netif_new"); return false; }

    sg_drv.base.post_attach = sg_post_attach;
    if (esp_netif_attach(sg_netif, &sg_drv) != ESP_OK) {
        Serial.println("[FATAL] esp_netif_attach"); return false;
    }

    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_WIFI_STA);
    mac[0] = 0x02; mac[5] ^= (ROLE_GATEWAY ? 0x11 : 0x22);
    esp_netif_set_mac(sg_netif, mac);

    esp_netif_action_start(sg_netif, NULL, 0, NULL);
    esp_netif_set_ip_info(sg_netif, &ip);
    esp_netif_action_connected(sg_netif, NULL, 0, NULL);

    Serial.printf("[NETIF] SG %s/%s gw %s\n", SG_IP, SG_MASK, SG_GW);
    return true;
}

static void napt_start() {
    // NAPT se habilita en la interfaz INTERNA (de donde viene el trafico privado)
    // Gateway: SG (10.0.0.0/30) es interna → STA (192.168.1.x) es externa
    // Cliente: AP (192.168.5.x) es interna → SG (10.0.0.0/30) es externa
#if ROLE_GATEWAY
    esp_netif_t *n = sg_netif;
    const char *w = "SG";
#else
    esp_netif_t *n = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    const char *w = "AP";
    esp_netif_set_default_netif(sg_netif);
#endif
    if (!n) { Serial.println("[NAPT] netif no encontrada"); return; }
    esp_err_t e = esp_netif_napt_enable(n);
    Serial.printf("[NAPT] %s -> %s\n", w, e == ESP_OK ? "OK" : esp_err_to_name(e));
}

// ============================================================
// 11. FRAGMENTACION
// ============================================================
static uint8_t frag_build_next() {
    if (!tx_cur) {
        sg_frame_t *f = NULL;
        if (xQueueReceive(q_tx, &f, 0) != pdTRUE) return 0;
        txq_bytes -= f->len;
        tx_cur = f; tx_off = 0;
    }
    uint16_t rem = tx_cur->len - tx_off;
    uint16_t n   = (rem > SG_FRAG_PAYLOAD) ? SG_FRAG_PAYLOAD : rem;

    uint8_t ctrl = SG_T_DATA;
    if (tx_off == 0)      ctrl |= SG_F_FIRST;
    if (rem > n)          ctrl |= SG_F_MF;

    tx_pkt[O_CTRL] = ctrl;
    memcpy(&tx_pkt[SG_HDR_LEN], tx_cur->data + tx_off, n);
    return SG_HDR_LEN + n;
}

static void frag_advance() {
    if (!tx_cur) return;
    tx_off += SG_FRAG_PAYLOAD;
    if (tx_off >= tx_cur->len) {
        st.eth_tx++; free(tx_cur); tx_cur = NULL; tx_off = 0;
    }
    tx_seq++; tx_retries = 0;
}

static void frag_abort() {
    if (tx_cur) { free(tx_cur); tx_cur = NULL; tx_off = 0; }
    st.eth_drop_arq++;
    tx_seq++; tx_retries = 0;
    reasm_active = false; reasm_len = 0;
}

static void reasm_push(const uint8_t *p, uint8_t total_len) {
    uint8_t ctrl = p[O_CTRL];
    uint8_t n    = total_len - SG_HDR_LEN;

    if (ctrl & SG_F_FIRST) { reasm_len = 0; reasm_active = true; }
    if (!reasm_active)     { st.reasm_err++; return; }
    if (reasm_len + n > SG_REASM_BUF) {
        st.reasm_err++; reasm_active = false; reasm_len = 0; return;
    }
    memcpy(reasm + reasm_len, p + SG_HDR_LEN, n);
    reasm_len += n;

    if (ctrl & SG_F_MF) return;

    sg_frame_t *f = (sg_frame_t*)malloc(sizeof(sg_frame_t) + reasm_len);
    if (f) {
        f->len = reasm_len; memcpy(f->data, reasm, reasm_len);
        if (xQueueSend(q_rx, &f, 0) != pdTRUE) { free(f); st.reasm_err++; }
        else st.eth_rx++;
    } else st.reasm_err++;
    reasm_active = false; reasm_len = 0;
}

// ============================================================
// 11b. BENCHMARK — genera trafico Ethernet dummy (FIX 5)
// ============================================================
static void benchmark_send() {
    uint8_t pkt[64];
    memset(pkt, 0, sizeof(pkt));

    memset(pkt, 0xFF, 6);
    pkt[6] = 0x02; pkt[7] = 0x00; pkt[8] = 0x00;
    pkt[9] = 0x00; pkt[10] = 0x00; pkt[11] = ROLE_GATEWAY ? 0x11 : 0x22;
    pkt[12] = 0x08; pkt[13] = 0x00;

    pkt[14] = 0x45;
    pkt[15] = 0x00;
    uint16_t ip_len = 20 + 8;
    pkt[16] = (ip_len >> 8) & 0xFF;
    pkt[17] = ip_len & 0xFF;
    pkt[18] = 0x00; pkt[19] = 0x00;
    pkt[20] = 0x00; pkt[21] = 0x00;
    pkt[22] = 64;
    pkt[23] = 0x01;
    if (ROLE_GATEWAY) {
        pkt[26] = 10; pkt[27] = 0; pkt[28] = 0; pkt[29] = 1;
        pkt[30] = 10; pkt[31] = 0; pkt[32] = 0; pkt[33] = 2;
    } else {
        pkt[26] = 10; pkt[27] = 0; pkt[28] = 0; pkt[29] = 2;
        pkt[30] = 10; pkt[31] = 0; pkt[32] = 0; pkt[33] = 1;
    }
    uint32_t ck = 0;
    for (int i = 14; i < 34; i += 2) ck += (pkt[i] << 8) | pkt[i+1];
    while (ck >> 16) ck = (ck & 0xFFFF) + (ck >> 16);
    pkt[24] = ~(ck >> 8) & 0xFF;
    pkt[25] = ~ck & 0xFF;

    pkt[34] = 8;
    pkt[35] = 0;
    uint32_t ic = 0;
    for (int i = 34; i < 42; i += 2) ic += (pkt[i] << 8) | pkt[i+1];
    while (ic >> 16) ic = (ic & 0xFFFF) + (ic >> 16);
    pkt[36] = ~(ic >> 8) & 0xFF;
    pkt[37] = ~ic & 0xFF;
    pkt[38] = 0; pkt[39] = 0;
    pkt[40] = 0; pkt[41] = 0;

    sg_transmit(NULL, pkt, 42);
}

// ============================================================
// 12. TASK MAC (core 1, prioridad alta)
// ============================================================
static inline void hdr_fill(uint8_t *p) {
    p[O_SEQ]   = tx_seq;
    p[O_ACK]   = rx_last_ok;
    p[O_RSSI]  = (int8_t)constrain((int)st.rssi_loc,  -128, 127);
    p[O_NOISE] = (int8_t)constrain((int)st.noise_loc, -128, 127);
    p[O_RSVD]  = 0;
}

static void mac_consume(const uint8_t *p, uint8_t len) {
    st.rssi_rem = (int8_t)p[O_RSSI];
    st.t_last_rx = millis(); st.up = true;

    if (tx_cur != NULL) {
        if (p[O_ACK] == tx_seq) {
            frag_advance();
        } else {
            tx_retries++; st.retx++;
            if (tx_retries > SG_MAX_RETRIES) frag_abort();
        }
    }

    if ((p[O_CTRL] & 0xC0) != SG_T_DATA) return;

    if (!rx_synced) {
        rx_synced = true;
        Serial.println("[MAC] RX synced (primer DATA recibido)");
    }

    tx_retries = 0;

    uint8_t seq = p[O_SEQ];

    if (seq == rx_expect) {
        reasm_push(p, len);
        rx_last_ok = seq; rx_expect++;
        st.frag_rx++; st.bytes_down += (len - SG_HDR_LEN);
    } else if (p[O_CTRL] & SG_F_FIRST) {
        Serial.printf("[MAC] desync rst: esperaba %u, llego %u (FIRST)\n",
                      rx_expect, seq);
        reasm_active = false; reasm_len = 0;
        rx_expect = seq + 1;
        rx_last_ok = seq;
        reasm_push(p, len);
        st.frag_rx++; st.bytes_down += (len - SG_HDR_LEN);
    } else if (seq == (uint8_t)(rx_expect - 1)) {
        rx_last_ok = seq;
    } else {
        st.reasm_err++;
        Serial.printf("[MAC] reasm_err: esperaba %u, llego %u\n",
                      rx_expect, seq);
    }
}

static void mac_task(void *arg) {
    uint32_t bm_timer = 0;
    for (;;) {
        if (do_selftest) { do_selftest = false; run_selftest(); }
        if (!running)    { vTaskDelay(pdMS_TO_TICKS(20)); continue; }

        if (benchmark && (millis() - bm_timer > 100)) {
            bm_timer = millis();
            benchmark_send();
        }

        uint8_t len = frag_build_next();
        if (len == 0) { tx_pkt[O_CTRL] = SG_T_NULL; len = SG_HDR_LEN; }
        hdr_fill(tx_pkt);

#if ROLE_GATEWAY
        uint32_t t0 = us_now();
        notify_flush();
        radio.startTransmit(tx_pkt, len);
        if (!wait_dio1(SG_GUARD_MS)) { radio.finishTransmit(); continue; }
        radio.finishTransmit();
        st.frag_tx++;
        if (len > SG_HDR_LEN) st.bytes_up += (len - SG_HDR_LEN);

        notify_flush();
        radio.startReceive();
        bool got = wait_dio1(SG_ACK_WAIT_MS);

        if (got) {
            float r = fsk_rssi();
            int e = radio.readData(rx_pkt, sizeof(rx_pkt));
            uint8_t rl = radio.getPacketLength();
            if (e == RADIOLIB_ERR_NONE && rl >= SG_HDR_LEN) {
                st.rssi_loc = r;
                mac_consume(rx_pkt, rl);
                cyc_sum += (us_now() - t0); cyc_n++;
            } else if (e == RADIOLIB_ERR_CRC_MISMATCH) {
                st.crc_err++; st.frag_lost++;
                if (st.frag_lost <= 3) Serial.printf("[RX] CRC err rl=%d seq=%d\n", rl, rx_pkt[1]);
                if (tx_cur) {
                    tx_retries++;
                    if (tx_retries > SG_MAX_RETRIES) frag_abort();
                }
            } else { st.frag_lost++; if(st.frag_lost <= 3) Serial.printf("[RX] readData err=%d rl=%d\n", e, rl); }
        } else {
            st.frag_lost++;
            radio.standby();
            if (tx_cur) {
                tx_retries++;
                if (tx_retries > SG_MAX_RETRIES) frag_abort();
            }
            if (audible) beep(2400, 10);
        }
#else
        notify_flush();
        radio.startReceive();
        if (!wait_dio1(1000)) { radio.standby(); continue; }

        float r = fsk_rssi();
        int e = radio.readData(rx_pkt, sizeof(rx_pkt));
        uint8_t rl = radio.getPacketLength();

        if (e == RADIOLIB_ERR_CRC_MISMATCH) {
            st.crc_err++; st.frag_lost++;
            if (audible) beep(2400, 10);
            continue;
        }
        if (e != RADIOLIB_ERR_NONE || rl < SG_HDR_LEN) { st.frag_lost++; continue; }

        st.rssi_loc = r;
        mac_consume(rx_pkt, rl);

        len = frag_build_next();
        if (len == 0) { tx_pkt[O_CTRL] = SG_T_NULL; len = SG_HDR_LEN; }
        hdr_fill(tx_pkt);

        notify_flush();
        radio.startTransmit(tx_pkt, len);
        if (wait_dio1(SG_GUARD_MS)) {
            st.frag_tx++;
            if (len > SG_HDR_LEN) st.bytes_up += (len - SG_HDR_LEN);
        }
        radio.finishTransmit();
#endif
    }
}

static void net_rx_task(void *arg) {
    sg_frame_t *f = NULL;
    for (;;) {
        if (xQueueReceive(q_rx, &f, portMAX_DELAY) == pdTRUE && f) {
            uint8_t *buf = (uint8_t*)malloc(f->len);
            if (buf) {
                memcpy(buf, f->data, f->len);
                if (esp_netif_receive(sg_netif, buf, f->len, buf) != ESP_OK) free(buf);
            }
            free(f);
        }
    }
}

// ============================================================
// 13. ESTADISTICAS
// ============================================================
static void stats_reset() {
    uint32_t t = millis();
    float keep_noise = st.noise_loc;
    memset((void*)&st, 0, sizeof(st));
    st.t0_ms = t; st.rssi_loc = -128; st.noise_loc = keep_noise;
    cyc_sum = 0; cyc_n = 0;
    tx_retries = 0; rx_synced = false;
    reasm_active = false; reasm_len = 0;
    rx_expect = 0; rx_last_ok = 0;
}

static void stats_update() {
    uint32_t tot = st.frag_rx + st.frag_lost;
    st.per = tot ? (100.0f * st.frag_lost / tot) : 0.0f;
    uint32_t dt = millis() - st.t0_ms;
    st.kbps_down = dt ? (st.bytes_down * 8.0f / dt) : 0.0f;
    st.kbps_up   = dt ? (st.bytes_up   * 8.0f / dt) : 0.0f;
    st.cycle_ms  = cyc_n ? ((float)cyc_sum / cyc_n / 1000.0f) : 0.0f;
    if (millis() - st.t_last_rx > 3000) st.up = false;
}

// ============================================================
// 14. WiFi
// ============================================================
static void wifi_start() {
    WiFi.persistent(false);
#if ROLE_GATEWAY
    WiFi.mode(WIFI_STA); WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);
    strncpy(net_ssid, WIFI_STA_SSID, 23); strncpy(net_pass, "(modo STA)", 23);
    for (int i = 0; i < 60 && WiFi.status() != WL_CONNECTED; i++) delay(250);
    strncpy(net_ip, WiFi.status() == WL_CONNECTED
            ? WiFi.localIP().toString().c_str() : "SIN CONEXION", 19);
#else
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192,168,5,1), IPAddress(192,168,5,1),
                      IPAddress(255,255,255,0));
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS, WIFI_AP_CHAN);
    strncpy(net_ssid, WIFI_AP_SSID, 23); strncpy(net_pass, WIFI_AP_PASS, 23);
    strncpy(net_ip, WiFi.softAPIP().toString().c_str(), 19);
#endif
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.printf("[WiFi] %s | %s\n", net_ssid, net_ip);
}

// ============================================================
// 15. EPD
// ============================================================
static void epd_pwr(bool on) { pca_w(PCA_EINK_EN, on); }

static void epd_body() {
    char b[28];
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);

    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(4, 15);
#if ROLE_GATEWAY
    display.print("GATEWAY/STA");
#else
    display.print("CLIENTE/AP");
#endif
    snprintf(b, sizeof(b), "%3d%%", bat_pct);
    display.setCursor(158, 15); display.print(b);
    display.drawLine(0, 20, 200, 20, GxEPD_BLACK);

    display.setFont(&FreeMono9pt7b);
    snprintf(b, sizeof(b), "SSID:%s", net_ssid); b[18] = 0;
    display.setCursor(4, 33); display.print(b);
    snprintf(b, sizeof(b), "IP  :%s", net_ip);   b[18] = 0;
    display.setCursor(4, 46); display.print(b);
    snprintf(b, sizeof(b), "SG  :%s", SG_IP);    b[18] = 0;
    display.setCursor(4, 59); display.print(b);
    display.drawLine(0, 64, 200, 64, GxEPD_BLACK);

    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(4, 78);
    if (!running) {
        display.print("DETENIDO");
    } else if (benchmark) {
        display.print("BENCHMARK");
    } else {
        display.print(st.up ? "LINK UP" : "BUSCANDO");
    }
#if !ROLE_GATEWAY
    snprintf(b, sizeof(b), "CLI:%d", WiFi.softAPgetStationNum());
    display.setCursor(128, 78); display.print(b);
#endif

    display.setFont(&FreeMono9pt7b);
    snprintf(b, sizeof(b), "CICLO %6.2f ms", st.cycle_ms);
    display.setCursor(4, 92);  display.print(b);
    snprintf(b, sizeof(b), "DOWN %6.1f kbps", st.kbps_down);
    display.setCursor(4, 106); display.print(b);
    snprintf(b, sizeof(b), "UP   %6.1f kbps", st.kbps_up);
    display.setCursor(4, 120); display.print(b);
    snprintf(b, sizeof(b), "COLA %4lu B  D%lu", txq_bytes, st.eth_drop_q);
    b[18] = 0; display.setCursor(4, 134); display.print(b);
    snprintf(b, sizeof(b), "PER  %6.2f %%", st.per);
    display.setCursor(4, 148); display.print(b);
    snprintf(b, sizeof(b), "RSSI %4.0f/%4d dBm", st.rssi_loc, st.rssi_rem);
    display.setCursor(4, 162); display.print(b);
    snprintf(b, sizeof(b), "ETH %lu/%lu R%lu", st.eth_tx, st.eth_rx, st.retx);
    b[18] = 0; display.setCursor(4, 176); display.print(b);

    snprintf(b, sizeof(b), "%s %s", running ? "B1:STOP " : "B1:START",
                                    audible ? "BEEP:ON" : "B2:BEEP");
    display.setCursor(4, 194); display.print(b);
}

static void epd_render(bool full = false) {
    static uint8_t pc = 0;
    stats_update();
    epd_pwr(true); delay(5);
    display.setRotation(0);
    if (full || pc++ >= 20) { display.setFullWindow(); pc = 0; }
    else                    { display.setPartialWindow(0, 0, 200, 200); }
    display.firstPage();
    do { epd_body(); } while (display.nextPage());
    epd_pwr(false);
}

static void ui_task(void *arg) {
    for (;;) {
        bat_pct = bat_read();
        epd_render();
        stats_update();
        Serial.printf("[STAT] ciclo=%.2f dn=%.1f up=%.1f PER=%.2f%% "
                      "cola=%lu ethTX=%lu ethRX=%lu dropQ=%lu dropARQ=%lu "
                      "retx=%lu reasm=%lu RSSI=%.0f/%d bm=%d\n",
                      st.cycle_ms, st.kbps_down, st.kbps_up, st.per,
                      txq_bytes, st.eth_tx, st.eth_rx, st.eth_drop_q,
                      st.eth_drop_arq, st.retx, st.reasm_err,
                      st.rssi_loc, st.rssi_rem, (int)benchmark);
        vTaskDelay(pdMS_TO_TICKS(running ? 6000 : 20000));
    }
}

// ============================================================
// 16. DIAGNÓSTICO DE RED
// ============================================================
static void cmd_netif_status() {
    if (!sg_netif) { Serial.println("[DIAG] sg_netif = NULL"); return; }
    esp_netif_t *n = esp_netif_next(NULL);
    while (n) {
        const char *key = esp_netif_get_ifkey(n);
        bool up = esp_netif_is_netif_up(n);
        esp_netif_ip_info_t ip;
        esp_netif_get_ip_info(n, &ip);
        uint8_t mac[6] = {0};
        bool mac_ok = (esp_netif_get_mac(n, mac) == ESP_OK);
        Serial.printf("[DIAG] %s | %s | IP %d.%d.%d.%d gw %d.%d.%d.%d",
                      key ? key : "?", up ? "UP" : "DOWN",
                      ip.ip.addr & 0xFF, (ip.ip.addr >> 8) & 0xFF,
                      (ip.ip.addr >> 16) & 0xFF, (ip.ip.addr >> 24) & 0xFF,
                      ip.gw.addr & 0xFF, (ip.gw.addr >> 8) & 0xFF,
                      (ip.gw.addr >> 16) & 0xFF, (ip.gw.addr >> 24) & 0xFF);
        if (mac_ok) Serial.printf(" | MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        else Serial.printf("\n");
        n = esp_netif_next(n);
    }
    esp_netif_t *def = esp_netif_get_default_netif();
    Serial.printf("[DIAG] Default: %s\n", def ? esp_netif_get_ifkey(def) : "NONE");
}

static void cmd_route_test() {
    uint32_t dst_ip = ROLE_GATEWAY ? esp_ip4addr_aton("10.0.0.2") : esp_ip4addr_aton("10.0.0.1");
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { Serial.printf("[TEST] socket errno=%d\n", errno); return; }
    struct sockaddr_in dst;
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = dst_ip;
    dst.sin_port = htons(9);
    uint32_t tx_before = st.eth_tx;
    int r = sendto(sock, "PING", 4, 0, (struct sockaddr*)&dst, sizeof(dst));
    if (r < 0) { Serial.printf("[TEST] sendto errno=%d\n", errno); }
    else {
        delay(50);
        uint32_t tx_after = st.eth_tx;
        Serial.printf("[TEST] UDP a %d.%d.%d.%d | eth_tx %lu -> %lu",
                      dst_ip & 0xFF, (dst_ip >> 8) & 0xFF,
                      (dst_ip >> 16) & 0xFF, (dst_ip >> 24) & 0xFF,
                      tx_before, tx_after);
        Serial.printf(tx_after > tx_before ? " RUTA OK\n" : " RUTA FALLIDA\n");
    }
    close(sock);
}

static void cmd_ping_to(const char *ip_str) {
    uint32_t dst_ip = esp_ip4addr_aton(ip_str);
    if (!dst_ip) { Serial.printf("[PING] IP invalida: %s\n", ip_str); return; }
    int sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (sock < 0) { Serial.printf("[PING] socket errno=%d\n", errno); return; }
    struct timeval tv = { 3, 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    struct sockaddr_in dst;
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = dst_ip;
    uint8_t pkt[64];
    memset(pkt, 0, sizeof(pkt));
    pkt[0] = 8;
    uint16_t id = (uint16_t)(millis() ^ (uint32_t)(intptr_t)&sock);
    pkt[4] = (id >> 8) & 0xFF; pkt[5] = id & 0xFF;
    pkt[6] = 1; pkt[7] = 0;
    uint32_t ck = 0;
    for (int i = 0; i < 8; i += 2) ck += (pkt[i] << 8) | pkt[i+1];
    while (ck >> 16) ck = (ck & 0xFFFF) + (ck >> 16);
    ck = ~ck; pkt[2] = (ck >> 8) & 0xFF; pkt[3] = ck & 0xFF;

    uint32_t t0 = millis();
    int r = sendto(sock, pkt, 8, 0, (struct sockaddr*)&dst, sizeof(dst));
    if (r < 0) { close(sock); Serial.printf("[PING] sendto errno=%d\n", errno); return; }
    Serial.printf("[PING] %s ... ", ip_str);
    uint8_t reply[128];
    struct sockaddr_in from;
    socklen_t fl = sizeof(from);
    r = recvfrom(sock, reply, sizeof(reply), 0, (struct sockaddr*)&from, &fl);
    if (r > 0) {
        uint32_t ip = from.sin_addr.s_addr;
        uint8_t ttl = (r >= 20) ? reply[8] : 0;
        uint8_t icmp_type = (r >= 21) ? reply[20] : 0;
        uint16_t rid = (r >= 23) ? ((uint16_t)reply[22] << 8) | reply[23] : 0;
        Serial.printf("r=%dms src=%d.%d.%d.%d ttl=%d icmp=%d id=0x%04x\n",
                      millis() - t0,
                      ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF,
                      ttl, icmp_type, rid);
    } else Serial.printf("timeout %lu ms\n", millis() - t0);
    close(sock);
}

static void cmd_forward_test() {
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (!ap) { Serial.println("[FWD] No hay AP netif (solo en cliente)"); return; }
    if (!esp_netif_is_netif_up(ap)) { Serial.println("[FWD] AP netif DOWN"); return; }

    uint8_t pkt[64];
    memset(pkt, 0, sizeof(pkt));
    memset(pkt, 0xFF, 6);
    pkt[6] = 0xAA; pkt[7] = 0xBB; pkt[8] = 0xCC;
    pkt[9] = 0xDD; pkt[10] = 0xEE; pkt[11] = 0x01;
    pkt[12] = 0x08; pkt[13] = 0x00;
    pkt[14] = 0x45; pkt[15] = 0x00;
    uint16_t ip_len = 20 + 8;
    pkt[16] = (ip_len >> 8) & 0xFF; pkt[17] = ip_len & 0xFF;
    pkt[18] = 0x00; pkt[19] = 0x01;
    pkt[20] = 0x00; pkt[21] = 0x00;
    pkt[22] = 64;
    pkt[23] = 0x01;
    pkt[26] = 192; pkt[27] = 168; pkt[28] = 5; pkt[29] = 10;
    pkt[30] = 10; pkt[31] = 0; pkt[32] = 0; pkt[33] = 1;
    uint32_t ck = 0;
    for (int i = 14; i < 34; i += 2) ck += (pkt[i] << 8) | pkt[i+1];
    while (ck >> 16) ck = (ck & 0xFFFF) + (ck >> 16);
    ck = ~ck; pkt[24] = (ck >> 8) & 0xFF; pkt[25] = ck & 0xFF;
    pkt[34] = 8; pkt[35] = 0;
    uint32_t ic = 0;
    for (int i = 34; i < 42; i += 2) ic += (pkt[i] << 8) | pkt[i+1];
    while (ic >> 16) ic = (ic & 0xFFFF) + (ic >> 16);
    ic = ~ic; pkt[36] = (ic >> 8) & 0xFF; pkt[37] = ic & 0xFF;
    pkt[38] = 0; pkt[39] = 1; pkt[40] = 0; pkt[41] = 1;

    uint32_t tx_before = st.eth_tx;
    uint8_t *buf = (uint8_t*)malloc(42);
    if (!buf) { Serial.println("[FWD] malloc fail"); return; }
    memcpy(buf, pkt, 42);
    esp_err_t e = esp_netif_receive(ap, buf, 42, buf);
    delay(100);
    uint32_t tx_after = st.eth_tx;
    Serial.printf("[FWD] AP inject %dB -> %s | eth_tx %lu->%lu\n",
                  42, e == ESP_OK ? "OK" : esp_err_to_name(e), tx_before, tx_after);
    if (tx_after > tx_before) Serial.println("[FWD] FORWARDING OK");
    else Serial.println("[FWD] FORWARDING FALLIDO");
}

static void cmd_dns_test() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { Serial.printf("[DNS] socket errno=%d\n", errno); return; }
    struct timeval tv = { 5, 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    struct sockaddr_in dst;
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = esp_ip4addr_aton("8.8.8.8");
    dst.sin_port = htons(53);
    uint8_t dns[] = {
        0xAA, 0xBB, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x06, 'g', 'o', 'o',
        'g', 'l', 'e', 0x03, 'c', 'o', 'm', 0x00,
        0x00, 0x01, 0x00, 0x01
    };
    uint32_t t0 = millis();
    uint32_t tx_before = st.eth_tx;
    int r = sendto(sock, dns, sizeof(dns), 0, (struct sockaddr*)&dst, sizeof(dst));
    if (r < 0) { close(sock); Serial.printf("[DNS] sendto errno=%d\n", errno); return; }
    uint8_t reply[256];
    struct sockaddr_in from;
    socklen_t fl = sizeof(from);
    r = recvfrom(sock, reply, sizeof(reply), 0, (struct sockaddr*)&from, &fl);
    uint32_t tx_after = st.eth_tx;
    if (r > 0) {
        uint32_t ip = from.sin_addr.s_addr;
        Serial.printf("[DNS] respuesta %lu ms desde %d.%d.%d.%d (%d B) eth_tx %lu->%lu\n",
                      millis() - t0,
                      ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF,
                      r, tx_before, tx_after);
    } else {
        Serial.printf("[DNS] timeout %lu ms eth_tx %lu->%lu (diff=%lu)\n",
                      millis() - t0, tx_before, tx_after, tx_after - tx_before);
    }
    close(sock);
}

static void cmd_ping_sg() {
    int sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
    if (sock < 0) { Serial.printf("[PING] socket errno=%d\n", errno); return; }
    struct timeval tv = { 2, 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    struct sockaddr_in dst;
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = ROLE_GATEWAY ? esp_ip4addr_aton("10.0.0.2") : esp_ip4addr_aton("10.0.0.1");
    uint8_t pkt[64];
    memset(pkt, 0, sizeof(pkt));
    pkt[0] = 8;
    pkt[4] = (uint8_t)(millis() >> 8); pkt[5] = (uint8_t)millis();
    pkt[6] = 1; pkt[7] = 0;
    uint32_t ck = 0;
    for (int i = 0; i < 8; i += 2) ck += (pkt[i] << 8) | pkt[i+1];
    while (ck >> 16) ck = (ck & 0xFFFF) + (ck >> 16);
    ck = ~ck; pkt[2] = (ck >> 8) & 0xFF; pkt[3] = ck & 0xFF;

    uint32_t t0 = millis();
    int r = sendto(sock, pkt, 8, 0, (struct sockaddr*)&dst, sizeof(dst));
    if (r < 0) { close(sock); Serial.printf("[PING] sendto errno=%d\n", errno); return; }
    Serial.printf("[PING] Echo a %d.%d.%d.%d ... ",
                  dst.sin_addr.s_addr & 0xFF, (dst.sin_addr.s_addr >> 8) & 0xFF,
                  (dst.sin_addr.s_addr >> 16) & 0xFF, (dst.sin_addr.s_addr >> 24) & 0xFF);
    uint8_t reply[128];
    struct sockaddr_in from;
    socklen_t fl = sizeof(from);
    r = recvfrom(sock, reply, sizeof(reply), 0, (struct sockaddr*)&from, &fl);
    if (r > 0) {
        uint32_t ip = from.sin_addr.s_addr;
        Serial.printf("respuesta %lu ms desde %d.%d.%d.%d\n", millis() - t0,
                      ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF);
    } else Serial.printf("timeout %lu ms\n", millis() - t0);
    close(sock);
}

// ============================================================
// 16a. COMANDOS SERIAL
// ============================================================
static void serial_cmd() {
    static char buf[32];
    static uint8_t i = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            buf[i] = 0; i = 0;
            if (strcmp(buf, "start") == 0) {
                running = true; stats_reset(); beep(1600, 80); epd_render(true);
                Serial.println("[CMD] START");
            } else if (strcmp(buf, "stop") == 0) {
                running = false; beep(700, 80); epd_render(true);
                Serial.println("[CMD] STOP");
            } else if (strcmp(buf, "bm") == 0) {
                benchmark = !benchmark; beep(benchmark ? 2000 : 800, 100);
                Serial.printf("[CMD] Benchmark %s\n", benchmark ? "ON" : "OFF");
                epd_render(true);
            } else if (strcmp(buf, "beep") == 0) {
                audible = !audible; beep(1200, 40);
                Serial.printf("[CMD] Beep %s\n", audible ? "ON" : "OFF");
            } else if (strcmp(buf, "reset") == 0 || strcmp(buf, "r") == 0) {
                stats_reset(); beep(900, 60); delay(40); beep(900, 60);
                Serial.println("[CMD] Stats reset");
            } else if (strcmp(buf, "selftest") == 0) {
                do_selftest = true;
                Serial.println("[CMD] Selftest");
            } else if (strcmp(buf, "stats") == 0) {
                stats_update();
                Serial.printf("ciclo=%.2f dn=%.1f up=%.1f PER=%.2f%% cola=%lu "
                              "ethTX=%lu ethRX=%lu dropQ=%lu dropARQ=%lu "
                              "retx=%lu reasm=%lu RSSI=%.0f/%d bm=%d\n",
                              st.cycle_ms, st.kbps_down, st.kbps_up, st.per,
                              txq_bytes, st.eth_tx, st.eth_rx, st.eth_drop_q,
                              st.eth_drop_arq, st.retx, st.reasm_err,
                              st.rssi_loc, st.rssi_rem, (int)benchmark);
            } else if (strcmp(buf, "netif") == 0) {
                cmd_netif_status();
            } else if (strcmp(buf, "route") == 0) {
                cmd_route_test();
            } else if (strcmp(buf, "ping") == 0) {
                cmd_ping_sg();
            } else if (strcmp(buf, "ping8") == 0) {
                cmd_ping_to("8.8.8.8");
            } else if (strcmp(buf, "ping1") == 0) {
                cmd_ping_to("1.1.1.1");
            } else if (strcmp(buf, "pinggw") == 0) {
                cmd_ping_to("192.168.1.1");
            } else if (strcmp(buf, "dns") == 0) {
                cmd_dns_test();
            } else if (strcmp(buf, "napt") == 0) {
                napt_start();
            } else if (strcmp(buf, "reboot") == 0) {
                Serial.println("[CMD] Rebooting..."); delay(100); esp_restart();
            } else if (strcmp(buf, "ipfwd") == 0) {
                Serial.printf("[FWD] ip_forward no accesible directamente\n");
            } else if (strlen(buf) > 0) {
                Serial.printf("[CMD] desconocido: %s\n", buf);
            }
        } else if (i < sizeof(buf) - 1) {
            buf[i++] = c;
        }
    }
}

// ============================================================
// 16. BOTONES
// ============================================================
static void buttons() {
    static bool p1 = HIGH, p2 = HIGH;
    static uint32_t d1 = 0, d2 = 0, t1 = 0, t2 = 0;
    static bool l1 = false, l2 = false;
    static uint32_t both_pressed = 0;

    bool c1 = digitalRead(PIN_BTN1);
    bool c2 = digitalRead(PIN_BTN2);

    if (c1 == LOW && c2 == LOW) {
        if (both_pressed == 0) both_pressed = millis();
        if (millis() - both_pressed > 3000) {
            both_pressed = 0;
            benchmark = !benchmark;
            beep(benchmark ? 2000 : 800, 100);
            Serial.printf("[BTN] Benchmark %s\n", benchmark ? "ON" : "OFF");
            epd_render(true);
            while (digitalRead(PIN_BTN1) == LOW || digitalRead(PIN_BTN2) == LOW) delay(10);
        }
        return;
    } else {
        both_pressed = 0;
    }

    if (c1 != p1 && millis() - d1 > DEBOUNCE_MS) {
        d1 = millis();
        if (c1 == LOW) { t1 = millis(); l1 = false; }
        else if (!l1) {
            running = !running;
            if (running) stats_reset();
            beep(running ? 1600 : 700, 80);
            epd_render(true);
        }
        p1 = c1;
    }
    if (p1 == LOW && !l1 && millis() - t1 > LONGPRESS_MS) {
        l1 = true; stats_reset(); beep(900, 60); delay(40); beep(900, 60);
        epd_render(true);
    }

    if (c2 != p2 && millis() - d2 > DEBOUNCE_MS) {
        d2 = millis();
        if (c2 == LOW) { t2 = millis(); l2 = false; }
        else if (!l2) { audible = !audible; beep(1200, 40); }
        p2 = c2;
    }
    if (p2 == LOW && !l2 && millis() - t2 > LONGPRESS_MS) {
        l2 = true;
        bool was = running; running = false;
        do_selftest = true; while (do_selftest) delay(50);
        running = was; epd_render(true);
    }
}

// ============================================================
// 17. SETUP / LOOP
// ============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println("\n=== SubGHz Bridge v1.2 BASELINE ===");

    pinMode(PIN_BTN1, INPUT_PULLUP);
    pinMode(PIN_BTN2, INPUT_PULLUP);
    pinMode(PIN_LORA_PWREN, OUTPUT); digitalWrite(PIN_LORA_PWREN, LOW);
    analogReadResolution(12); analogSetAttenuation(ADC_11db);
    ledcAttach(PIN_BUZZER, 2000, 8);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL); Wire.setClock(400000);
    if (!pca_init()) { beep(300, 2000); while (1) delay(1000); }
    pca_w(PCA_POWER_EN, true); pca_w(PCA_LED_ENABLE, true); delay(50);

    spi_lora.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    spi_epd .begin(PIN_EPD_SCK,  -1,            PIN_EPD_MOSI,  PIN_EPD_CS);
    epd_pwr(true); delay(10);
    display.epd2.selectSPI(spi_epd, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    display.init(115200, true, 2, false);
    epd_pwr(false);

    if (!radio_init()) { beep(300, 2000); while (1) delay(1000); }
    stats_reset();

    q_tx = xQueueCreate(SG_TXQ_DEPTH, sizeof(sg_frame_t*));
    q_rx = xQueueCreate(SG_RXQ_DEPTH, sizeof(sg_frame_t*));
    if (!q_tx || !q_rx) { Serial.println("[FATAL] colas"); while (1) delay(1000); }

    xTaskCreatePinnedToCore(mac_task, "mac", 8192, NULL, 10, &h_mac, 1);
    delay(50);

    st.noise_loc = measure_noise();
    Serial.printf("[RF] Piso de ruido = %.1f dBm\n", st.noise_loc);
    do_selftest = true; while (do_selftest) delay(50);

    wifi_start();
    if (!sg_netif_start()) { beep(300, 2000); while (1) delay(1000); }
    napt_start();

    xTaskCreatePinnedToCore(net_rx_task, "netrx", 6144, NULL, 8, NULL, 0);
    xTaskCreatePinnedToCore(ui_task,     "ui",    8192, NULL, 1, NULL, 0);

    bat_pct = bat_read();
    epd_render(true);
    beep(1000, 60); delay(40); beep(1600, 90);
    Serial.println("[INIT] listo. B1=START/STOP  B1+B2>3s=BM  CMD:netif/route/ping/ping8/dns/reboot");
}

void loop() { buttons(); serial_cmd(); delay(10); }
