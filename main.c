#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "boards.h"
#include "app_timer.h"
#include "custom_gpio_drivers.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

#include "bsp.h"
#include "bsp_btn_ble.h"

#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

#include "generic_level_server.h"
#include "scene_setup_server.h"
#include "model_config_file.h"
#include "generic_level_client.h"

#include "log.h"
#include "rtt_input.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_mesh_config_app.h"
#include "nrf.h"
#include "example_common.h"
#include "pwm_utils.h"
#include "nrf_mesh_config_examples.h"
#include "app_level.h"
#include "app_dtt.h"
#include "ble_softdevice_support.h"
#include "nrf_delay.h"

#include "nrf_drv_wdt.h"

#include "nrf_drv_saadc.h"
#define MQ2_ANALOG_PIN NRF_SAADC_INPUT_AIN0

#include "st7789_drivers.h"
#include "nrf52_timer.h"
#include "nrf52_at24c32_64.h"
#include "nrf52_aht10.h"
#include "DHT.h"

#include "app_uart.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define TRIG_PIN        11
#define TRIG_WIDTH_US   200

static inline void trig(uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
        nrf_gpio_pin_set(TRIG_PIN);
        nrf_delay_us(TRIG_WIDTH_US);
        nrf_gpio_pin_clear(TRIG_PIN);
        nrf_delay_us(TRIG_WIDTH_US);
    }
}

extern uint32_t m_secondCounter;

static inline uint32_t get_local_time_us(void)
{
    static uint32_t last_ticks = 0;
    static uint64_t acc_ticks  = 0;
    uint32_t now, diff;

    CRITICAL_REGION_ENTER();
    now  = app_timer_cnt_get() & 0x00FFFFFF;
    diff = (now - last_ticks) & 0x00FFFFFF;
    last_ticks = now;
    acc_ticks += diff;
    CRITICAL_REGION_EXIT();

    return (uint32_t)((acc_ticks * 1000000ULL) / 32768ULL);
}

enum signalStatus   
{
    _normalStatus = 1,
    _fireAlarmStatus = 3,
};

static uint32_t m_present_level;
uint32_t timerBCounter;
uint32_t refreshDisplayCounter = 1;
bool _getResetProvisionKey = false;

volatile uint16_t g_last_mesh_src_addr = 0;
volatile int8_t   g_last_mesh_rssi     = 0;
volatile uint8_t  g_last_mesh_ttl      = 0;

bool clientProvisioned = false;
bool clientProvisionChanged = false;

static void trigger_egu30_event(void);

bool aht10Available;
float _clientTemperature;
float _clientHumidity;

bool mq2Available = true;
uint16_t _clientMQ2Value;

bool clearAllDisplayNodes;

#define UART_TX_BUF_SIZE        256                   
#define UART_RX_BUF_SIZE        256                   

void uart_error_handle(app_uart_evt_t * p_event)
{
    /* B? qua toàn b? l?i UART, không g?i APP_ERROR_HANDLER */
    (void)p_event;
}

dsm_local_unicast_address_t node_address;

void uart_puts(const char *_ch)
{
    while (*_ch)
    {
        if (app_uart_put(*_ch) != NRF_SUCCESS)
        {
            break;
        }
        _ch++;
    }
}

void uart_init(void)
{
    uint32_t err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART_PIN_DISCONNECTED,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = UART_PIN_DISCONNECTED,
        .cts_pin_no   = UART_PIN_DISCONNECTED,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
    #if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
    #else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
    #endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

#define LOG_QUEUE_SIZE        16
#define LOG_MSG_MAX_LEN       128

static char m_log_queue[LOG_QUEUE_SIZE][LOG_MSG_MAX_LEN];
static volatile uint8_t m_log_head = 0;
static volatile uint8_t m_log_tail = 0;
static volatile bool m_logging_enabled = true;
bool periodic_message_allowed = false;

static void log_uart_mesh_event(const char* event, uint16_t src, uint16_t dst, 
                                uint32_t seq, uint8_t ttl_tx, uint8_t ttl_rx, 
                                int8_t rssi, const char* traffic_class, uint8_t len)
{
    if (!m_logging_enabled)
    {
        return;
    }
    uint32_t t_us = get_local_time_us();
    uint16_t my_id = node_address.address_start & 0xFFFF;
    
    uint8_t next_head = (m_log_head + 1) % LOG_QUEUE_SIZE;
    
    if (next_head != m_log_tail)
    {
        snprintf(m_log_queue[m_log_head], LOG_MSG_MAX_LEN, 
                 "%u,0x%04X,%s,0x%04X,0x%04X,%u,%u,%u,%d,%s,%u\r\n",
                 t_us, my_id, event, src, dst, seq, ttl_tx, ttl_rx, rssi, traffic_class, len);

        SEGGER_RTT_WriteString(0, m_log_queue[m_log_head]);

        m_log_head = next_head;
    }
}

static void log_queue_flush(void)
{
    while (m_log_tail != m_log_head)
    {
        uart_puts(m_log_queue[m_log_tail]);
        m_log_tail = (m_log_tail + 1) % LOG_QUEUE_SIZE;
    }
}

extern nrfx_wdt_channel_id m_channel_id;

#define MAX_OFFLINE_RECORDS  1800

typedef struct __attribute__((packed)) {
    uint32_t t_us;
    uint16_t seq_alarm;
    uint16_t src;
    uint8_t  ttl;
    int8_t   rssi;
} offline_log_t;

static offline_log_t m_offline_logs[MAX_OFFLINE_RECORDS];
static volatile uint16_t m_offline_count = 0;
static volatile uint16_t m_offline_dropped = 0;
static volatile bool m_dump_requested = false;
volatile uint8_t g_display_src = 0;

static bool rtt_write_line_reliable(const char *p, unsigned len)
{
    uint32_t waited_ms = 0;
    while (SEGGER_RTT_Write(0, p, len) != len)
    {
        nrf_drv_wdt_channel_feed(m_channel_id);
        nrf_delay_ms(1);
        if (++waited_ms >= 3000)
        {
            return false;
        }
    }
    return true;
}

void offline_log_push_tx(uint16_t seq, uint8_t ttl, bool is_alarm)
{
    if (!m_logging_enabled)
    {
        return;
    }
    if (m_offline_count >= MAX_OFFLINE_RECORDS)
    {
        m_offline_dropped++;
        return;
    }

    m_offline_logs[m_offline_count].t_us        = get_local_time_us();
    m_offline_logs[m_offline_count].seq_alarm   = ((is_alarm ? 1 : 0) << 15) | (seq & 0x7FFF);
    m_offline_logs[m_offline_count].src         = (uint16_t)(node_address.address_start & 0xFFFF);
    m_offline_logs[m_offline_count].ttl         = ttl;
    m_offline_logs[m_offline_count].rssi        = 127;
    m_offline_count++;
}

void offline_log_push(uint16_t src, uint8_t ttl, int8_t rssi, uint16_t payload)
{
    if (!m_logging_enabled)
    {
        return;
    }

    trig(3); 

    uint16_t raw_seq     = payload & 0x7FFF;
    uint8_t  traffic_bit = (payload >> 15) & 0x01;

    uint16_t clean_src = src;
    if (clean_src % 2 == 0 && clean_src > 1)
    {
        clean_src -= 1;
    }
    g_display_src = (uint8_t)(clean_src & 0xFF);

    if (m_offline_count < MAX_OFFLINE_RECORDS)
    {
        m_offline_logs[m_offline_count].t_us        = get_local_time_us();
        m_offline_logs[m_offline_count].seq_alarm   = (traffic_bit << 15) | raw_seq;
        m_offline_logs[m_offline_count].src         = clean_src;
        m_offline_logs[m_offline_count].ttl         = ttl;
        m_offline_logs[m_offline_count].rssi        = (rssi == 127) ? 126 : rssi;
        m_offline_count++;
    }
    else
    {
        m_offline_dropped++;
    }

    log_uart_mesh_event("RX", clean_src, node_address.address_start, 
                        raw_seq, 0, ttl, rssi, 
                        (traffic_bit ? "ALARM" : "TELEMETRY"), sizeof(payload));
}

static void dump_offline_logs_to_uart(void)
{
    char buf[128];
    bool aborted = false;

    m_logging_enabled = false;
    periodic_message_allowed = false;

    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    int len = snprintf(buf, sizeof(buf),
                       "\r\n--- BAT DAU XA LOG CSV --- SO DONG: %u; BI TRAN BUFFER: %u ---\r\n",
                       (unsigned)m_offline_count, (unsigned)m_offline_dropped);
    if (len > 0 && !rtt_write_line_reliable(buf, (unsigned)len))
    {
        aborted = true;
    }

    for (uint16_t i = 0; (i < m_offline_count) && !aborted; i++)
    {
        nrf_drv_wdt_channel_feed(m_channel_id);

        uint16_t seq       = m_offline_logs[i].seq_alarm & 0x7FFF;
        bool     is_alarm  = (m_offline_logs[i].seq_alarm >> 15) & 0x01;
        uint16_t node_id   = m_offline_logs[i].src;
        uint8_t  ttl_val   = m_offline_logs[i].ttl;

        bool is_tx         = (m_offline_logs[i].rssi == 127);
        const char *evt_str = is_tx ? "TX" : "RX";

        uint16_t src       = is_tx ? (node_address.address_start & 0xFFFF) : node_id;
        uint16_t dst       = is_tx ? 0xFFFF : (node_address.address_start & 0xFFFF);
        uint8_t  ttl_tx    = is_tx ? ttl_val : 0;
        uint8_t  ttl_rx    = is_tx ? 0 : ttl_val;
        int8_t   real_rssi = is_tx ? 0 : m_offline_logs[i].rssi;

        len = snprintf(buf, sizeof(buf),
                       "%u,0x%04X,%s,0x%04X,0x%04X,%u,%u,%u,%d,%s,2\r\n",
                       m_offline_logs[i].t_us,
                       node_address.address_start & 0xFFFF,
                       evt_str,
                       src,
                       dst,
                       seq,
                       ttl_tx,
                       ttl_rx,
                       real_rssi,
                       (is_alarm ? "ALARM" : "TELEMETRY"));

        if (len > 0 && !rtt_write_line_reliable(buf, (unsigned)len))
        {
            aborted = true;
        }
    }

    if (aborted)
    {
        SEGGER_RTT_WriteString(0, "\r\n[LOI] RTT chua mo hoac bi nghen! Du lieu van giu trong RAM, hay thu bam Nut 3 lai.\r\n");
        return;
    }

    (void)rtt_write_line_reliable("--- KET THUC XA LOG ---\r\n", 25);

    m_offline_count = 0;
    m_offline_dropped = 0;
    m_logging_enabled = true;
}

static void log_config_init(void)
{
    SEGGER_RTT_Init();
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    if (err_code == NRF_SUCCESS)
    {
        NRF_LOG_DEFAULT_BACKENDS_INIT();
    }
    SEGGER_RTT_WriteString(0, "\r\n\r\n====== RTT DA KET NOI THANH CONG ======\r\n\r\n");
}

void mq2_saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(MQ2_ANALOG_PIN);
    err_code = nrf_drv_saadc_init(NULL, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
}

uint16_t read_mq2_sensor(void)
{
    nrf_saadc_value_t val;
    nrf_drv_saadc_sample_convert(0, &val);
    if (val < 0) val = 0;
    return (uint16_t)val;
}

#define APP_LEVEL_ELEMENT_INDEX       (0)
#define APP_FORCE_SEGMENTATION        (false)
#define APP_MIC_SIZE                  (NRF_MESH_TRANSMIC_SIZE_SMALL)
#define CLIENT_MODEL_INSTANCE_COUNT   (1)
#define APP_LEVEL_DELAY_MS            (100)
#define APP_LEVEL_TRANSITION_TIME_MS  (10)

static void app_generic_level_client_status_cb(const generic_level_client_t * p_self, const access_message_rx_meta_t * p_meta, const generic_level_status_params_t * p_in);
static void app_gen_level_client_transaction_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status);
static void app_gen_level_client_publish_interval_cb(access_model_handle_t handle, void * p_self);

static generic_level_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];

const generic_level_client_callbacks_t client_cbs =
{
    .level_status_cb = app_generic_level_client_status_cb,
    .ack_transaction_status_cb = app_gen_level_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_level_client_publish_interval_cb
};

static void app_generic_level_client_status_cb(const generic_level_client_t * p_self, const access_message_rx_meta_t * p_meta, const generic_level_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Level server: 0x%04x, Present Level: %d, Target Level: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_level, p_in->target_level, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Level server: 0x%04x, Present Level: %d\n", p_meta->src.value, p_in->present_level);
    }
}

static void app_gen_level_client_transaction_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status)
{
    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;
        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;
        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

static void app_gen_level_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

static void alarm_status_check(void)
{
    if (nrf_gpio_pin_read(BSP_BUTTON_0) == false)
    {
        m_present_level = _fireAlarmStatus;    
        bsp_board_led_on(BSP_BOARD_LED_3);
    }
    else
    {
        m_present_level = _normalStatus;
        bsp_board_led_off(BSP_BOARD_LED_3);
    }
}

static uint16_t m_tx_packet_count = 0;
static uint32_t m_global_seq = 0;

static void publish_present_alarm_status(bool _statusSending)
{
    if (!_statusSending && m_tx_packet_count >= 1500)
    {
        periodic_message_allowed = false;
        return;
    }

    uint8_t client = 0;
    static generic_level_set_params_t set_params = {0}; 
    model_transition_t transition_params;

    transition_params.delay_ms = APP_LEVEL_DELAY_MS;
    transition_params.transition_time_ms = APP_LEVEL_TRANSITION_TIME_MS;

    const char *p_class = "TELEMETRY";
    uint16_t traffic_bit = 0;

    if (_statusSending) 
    {
        traffic_bit = 1;
        p_class = "ALARM";
    }

    static uint8_t m_tid_counter = 0;
    uint32_t next_seq = m_global_seq + 1;
    uint16_t seq_15bit = (uint16_t)(next_seq & 0x7FFF);

    uint16_t payload = (traffic_bit << 15) | seq_15bit;

    set_params.level = (int16_t)payload;
    set_params.tid   = m_tid_counter++;

    (void)access_model_reliable_cancel(m_clients[client].model_handle);

    trig(1);

    uint32_t status = generic_level_client_set(&m_clients[client], &set_params, &transition_params);

    if (status == NRF_SUCCESS)
    {
        m_global_seq = next_seq;

        if (!_statusSending)
        {
            m_tx_packet_count++;
        }

        uint16_t dst_addr = 0xFFFF;
        uint8_t current_ttl = 0;

        (void)access_model_publish_ttl_get(m_clients[client].model_handle, &current_ttl);

        offline_log_push_tx(seq_15bit, current_ttl, _statusSending);

        log_uart_mesh_event("TX", node_address.address_start, dst_addr, 
                            (uint32_t)seq_15bit, current_ttl, 0, 0, p_class, sizeof(payload));
    }
    else if (status != NRF_ERROR_NO_MEM && status != NRF_ERROR_BUSY && 
             status != NRF_ERROR_INVALID_STATE && status != NRF_ERROR_INVALID_PARAM)
    {
        ERROR_CHECK(status);
    }
}

static void execute_periodic_message_sending(void)
{
    if (periodic_message_allowed)
    {
        periodic_message_allowed = false;
    }
    else
    {
        m_tx_packet_count = 0;
        m_global_seq = 0;
        periodic_message_allowed = true;
    }
}
static void mesh_main_button_event_handler(uint32_t button_number)
{
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    switch (button_number)
    {
        case 1:
            alarm_status_check();
            trigger_egu30_event();
            break;
        case 2:
            execute_periodic_message_sending();
            break;
        case 3:
            m_dump_requested = true;
            break;
        case 4:
            clearAllDisplayNodes = true;
            break;
        default:
            break;
    }
}

void bsp_event_handler(bsp_event_t event)
{
    switch (event)
    {
        case BSP_EVENT_KEY_0:
        case BSP_EVENT_KEY_1:
        case BSP_EVENT_KEY_2:
        case BSP_EVENT_KEY_3:
            mesh_main_button_event_handler(event - BSP_EVENT_KEY_0);
            break;
        default:
            break;
    }
}

static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;
    uint32_t err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

static void models_client_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;
        ERROR_CHECK(generic_level_client_init(&m_clients[i], i + 1));
    }
}

static void mesh_events_handle(const nrf_mesh_evt_t * p_evt);
static void app_level_server_set_cb(const app_level_server_t * p_server, uint32_t present_level);
static void app_level_server_get_cb(const app_level_server_t * p_server, uint32_t * p_present_level);
static void app_level_server_transition_cb(const app_level_server_t * p_server, uint32_t transition_time_ms, uint32_t target_level, app_transition_type_t transition_type);

static nrf_mesh_evt_handler_t m_event_handler =
{
    .evt_cb = mesh_events_handle,
};

APP_LEVEL_SERVER_DEF(m_level_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     NULL,
                     app_level_server_set_cb,
                     app_level_server_get_cb,
                     app_level_server_transition_cb);

/* ========================================================================== */
/* KH?I X? LÝ NH?N GÓI MESH CHU?N HÓA CHO THÍ NGHI?M VÀ HI?N TH?             */
/* ========================================================================== */

static void app_level_server_set_cb(const app_level_server_t * p_server, uint32_t present_level)
{
    uint16_t payload     = (uint16_t)present_level;
    uint8_t  traffic_bit = (payload >> 15) & 0x01;
    uint16_t seq_num     = payload & 0x7FFF;

    /* Chu?n hóa d?a ch? Element 1 v? Node ID chính d? hi?n th? */
    uint16_t raw_src = g_last_mesh_src_addr;
    uint16_t src_addr = raw_src;
    if (raw_src % 2 == 0 && raw_src > 1)
    {
        src_addr = raw_src - 1;
    }

    /* 1. IN LOG DEBUG Ð?P M?T RA RTT VIEWER */
    SEGGER_RTT_printf(0, "[RX] Node: 0x%04X | %s | Seq: %u | RSSI: %d\r\n",
                      src_addr,
                      (traffic_bit == 1) ? "ALARM" : "TELEMETRY",
                      seq_num,
                      g_last_mesh_rssi);

    /* 2. C?P NH?T GIAO DI?N MÀN HÌNH ST7789 */
    uint32_t display_stat = (traffic_bit == 1) ? _fireAlarmStatus : _normalStatus;
    insert_node((uint8_t)(src_addr & 0xFF), display_stat, false);

    refreshDisplayCounter = 1;
}

static void app_level_server_get_cb(const app_level_server_t * p_server, uint32_t * p_present_level)
{
    *p_present_level = (node_address.address_start << 16) + 6;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_level_server_Get_cb\n\r");
}

static void app_level_server_transition_cb(const app_level_server_t * p_server, 
                                           uint32_t transition_time_ms, 
                                           uint32_t target_level, 
                                           app_transition_type_t transition_type)
{
    /* Ð? TR?NG: Toàn b? vi?c ghi log dã hoàn t?t ? Access Layer */
}

static void app_model_init(void)
{
    ERROR_CHECK(app_level_init(&m_level_server_0, APP_LEVEL_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Level Model handle: %d\n", m_level_server_0.server.model_handle);
}

static void mesh_events_handle(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_ENABLED)
    {
        APP_ERROR_CHECK(app_level_value_restore(&m_level_server_0));
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    model_config_file_clear();
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, false);
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
    clientProvisionChanged = true;
    clientProvisioned = true;
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");
#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif
    unicast_address_print();
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
    models_client_init_cb();
}

static void mesh_init(void)
{
    model_config_file_init();

    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,  
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);

    if (status == NRF_SUCCESS)
    {
        status = model_config_file_config_apply();
    }

    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            model_config_file_clear();
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in persistent memory corrupted. Device starts unprovisioned.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP, LOG_LEVEL_WARN, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Sensor Node -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

    nrf_gpio_cfg_output(TRIG_PIN);
    nrf_gpio_pin_clear(TRIG_PIN);

    bool erase_bonds;
    buttons_leds_init(&erase_bonds);
    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    hal_led_blink_stop();
    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);    
    mesh_init();
}

static void start(void)
{
    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_DM_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));

        clientProvisionChanged = true;
        clientProvisioned = false;
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());
    nrf_mesh_evt_handler_add(&m_event_handler);
    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(BLUE_LED_pos | YELLOW_LED_pos, LED_MASK_STATE_OFF);
    hal_led_blink_ms(BLUE_LED_pos | YELLOW_LED_pos, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

static void trigger_egu30_event(void)
{
    NRF_EGU3->TASKS_TRIGGER[0] = 1;
}

static void all_parameters_initialization(void)
{
    m_present_level = _normalStatus;       
    timerBCounter = 1;
    periodic_message_allowed = false;      
    aht10Available = true;
    _clientTemperature = 0;
    _clientHumidity = 0;
    clearAllDisplayNodes = false;
}

void SWI3_EGU3_IRQHandler(void)
{   
    NRF_EGU3->EVENTS_TRIGGERED[0] = 0;

    if (_getResetProvisionKey)
    {
        if (mesh_stack_is_device_provisioned())
        {
            mesh_config_clear();
            node_reset();
        }
    }

    if (m_present_level == _fireAlarmStatus)
    {
        publish_present_alarm_status(true);
    }
    else if (periodic_message_allowed)
    {
        publish_present_alarm_status(false);
    }
}

static void egu_init(void)
{  
    NRF_EGU3->INTENSET = EGU_INTENSET_TRIGGERED0_Enabled << EGU_INTENSET_TRIGGERED0_Pos;
    NVIC_SetPriority(SWI3_EGU3_IRQn, 6);
    NVIC_EnableIRQ(SWI3_EGU3_IRQn);
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    error_info_t * p_info = (error_info_t *)info;
    char err_buf[128];

    if (p_info != NULL && id == NRF_FAULT_ID_SDK_ERROR)
    {
        snprintf(err_buf, sizeof(err_buf), 
                 "\r\n[FAULT] Code: 0x%08X, File: %s, Line: %u\r\n", 
                 (unsigned int)p_info->err_code, p_info->p_file_name, (unsigned int)p_info->line_num);
    }
    else
    {
        snprintf(err_buf, sizeof(err_buf), 
                 "\r\n[FAULT] ID: 0x%08X, PC: 0x%08X, INFO: 0x%08X\r\n", 
                 (unsigned int)id, (unsigned int)pc, (unsigned int)info);
    }
    uart_puts(err_buf);
    NRF_LOG_RAW_INFO("%s", err_buf);
    
    while (1);
}

nrfx_wdt_channel_id m_channel_id;

void wd_event_handle(void)
{
}

void wdt_init(void)
{
    uint32_t err_code = NRF_SUCCESS;  
    nrfx_wdt_config_t w_config = NRFX_WDT_DEAFULT_CONFIG; 
    err_code = nrfx_wdt_init(&w_config, wd_event_handle); 
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_wdt_channel_alloc(&m_channel_id); 
    APP_ERROR_CHECK(err_code);
    nrfx_wdt_enable(); 
}

#define resetProvisionLED     18    
#define resetProvisionBTN     13

bool checktoResetProvisioned(void)
{
    nrf_gpio_cfg_output(resetProvisionLED);
    nrf_gpio_cfg_input(resetProvisionBTN, NRF_GPIO_PIN_PULLUP);
    bool _checkresetProvision = false;
    uint64_t resetProvisionCounter = 0;
    while (!_checkresetProvision)
    {
        nrf_delay_ms(1);
        if (nrf_gpio_pin_read(resetProvisionBTN) == 0)
        {
            nrf_gpio_pin_clear(resetProvisionLED);
            resetProvisionCounter++;
            if (resetProvisionCounter > 5000)
            {
                _checkresetProvision = true;
            }
        }
        else
        {
            break;
        }
    }
    return _checkresetProvision;
}

void display_ClientStatus(void)
{
    if (clientProvisionChanged)
    {
        if (clientProvisioned)
        {  
            char _buff[100];
            sprintf(_buff, "0x%04x", node_address.address_start);
            glcd_display_stationAddress(_buff);
        }
        else
        {
            clearSavedExtEEPROMNodes();
            clearNodesScreen();
            display_nodes();
            glcd_display_stationAddress("------");
        }
        clientProvisionChanged = false;
    }
}

int main(void)
{
    log_config_init();
    SEGGER_RTT_WriteString(0, "-> Qua log_config_init\r\n");

    _getResetProvisionKey = checktoResetProvisioned(); // CHÚ Ý: Ch? này có while l?p!
    SEGGER_RTT_WriteString(0, "-> Qua checktoResetProvisioned\r\n");

    all_parameters_initialization(); //
    uart_init(); //
    SEGGER_RTT_WriteString(0, "-> Qua uart_init\r\n");

    mq2_saadc_init(); //
    SEGGER_RTT_WriteString(0, "-> Qua mq2_saadc_init\r\n");

    glcd_display_initialization(); //
    SEGGER_RTT_WriteString(0, "-> Qua glcd_init\r\n");

    glcd_printText("Nodes:", 1, 40, TFT_WHITE, TFT_BLACK); //
    SEGGER_RTT_WriteString(0, "-> Qua glcd_printText\r\n");

    initialize(); //
    SEGGER_RTT_WriteString(0, "-> Qua initialize\r\n");

    start(); //[cite: 2]
    SEGGER_RTT_WriteString(0, "-> Qua start (Mesh da chay)\r\n");

    wdt_init();
    SEGGER_RTT_WriteString(0, "-> Qua wdt_init\r\n");

    custom_timer_init();
    SEGGER_RTT_WriteString(0, "-> Qua custom_timer_init\r\n");

    egu_init();
    SEGGER_RTT_WriteString(0, "-> Qua egu_init\r\n");

    twi_init();
    SEGGER_RTT_WriteString(0, "-> Qua twi_init\r\n");

    static uint8_t _i2cDevices[255];
    detect_i2cDevice(_i2cDevices);
    SEGGER_RTT_WriteString(0, "-> Qua detect_i2cDevice\r\n");

    loadExtEEPROM_Nodes();
    SEGGER_RTT_WriteString(0, "-> Qua loadExtEEPROM_Nodes\r\n");

    nrf_drv_wdt_channel_feed(m_channel_id);

    if (aht10_begin() == true)
    {
        aht10Available = true;
    }
    SEGGER_RTT_WriteString(0, "-> Qua aht10_begin\r\n");

    if (mq2Available)
    {
        _clientMQ2Value = read_mq2_sensor();
    }

    if (aht10Available)
    {
        _clientTemperature = aht10_readTemperature(AHT10_FORCE_READ_DATA);
        _clientHumidity = aht10_readHumidity(false); 
        glcd_printSensor(_clientTemperature, _clientHumidity, _clientMQ2Value);
    }

    if (_getResetProvisionKey)
    {
        trigger_egu30_event();
    }

    SEGGER_RTT_WriteString(0, "-> Vao vong lap for(;;) thanh cong!\r\n");

    for (;;)
    {
        /* 1. X? lý x? log Offline khi nh?n Nút 3 */
        if (m_dump_requested)
        {
            SEGGER_RTT_WriteString(0, "[ACTION] Nhan Nut 3 -> Dang xa log...\r\n");
            dump_offline_logs_to_uart();
            m_dump_requested = false;
        }

        /* 2. X? hàng d?i UART n?u có d? li?u t?n d?ng */
        log_queue_flush();

        /* 3. X? lý s? ki?n m?i giây */
        if (m_second_changed)
        {
            char rtt_buf[64];
            snprintf(rtt_buf, sizeof(rtt_buf), "[TICK] Giay: %u | Prov: %s\r\n", 
                     m_secondCounter, 
                     mesh_stack_is_device_provisioned() ? "DA PROVISION" : "CHUA PROVISION");
            SEGGER_RTT_WriteString(0, rtt_buf);

            glcd_integer_print(m_secondCounter);
            display_ClientStatus();

            if (clearAllDisplayNodes)
            {
                clearSavedExtEEPROMNodes();
                clearNodesScreen();
                display_nodes();
                clearAllDisplayNodes = false;
            }

            bool is_timeout = check_connection_nodes();
            bool is_poll_5s = (m_secondCounter % 5 == 0);

            if (!is_timeout && is_poll_5s)
            {
                if (mq2Available)
                {
                    _clientMQ2Value = read_mq2_sensor();
                }

                if (aht10Available)
                {
                    _clientTemperature = aht10_readTemperature(AHT10_FORCE_READ_DATA);
                    _clientHumidity = aht10_readHumidity(false); 
                    glcd_printSensor(_clientTemperature, _clientHumidity, _clientMQ2Value);
                }
                else
                {
                    aht10Available = aht10_begin();
                }
            }

            if ((refreshDisplayCounter > 0) || is_timeout || is_poll_5s)
            {
                display_nodes();
                refreshDisplayCounter = 0;
            }

            /* N?u chua provision thì chua phát tin periodic */
            if (periodic_message_allowed && mesh_stack_is_device_provisioned())
            {
                trigger_egu30_event();
            }

            m_second_changed = false;
        }

        /* 4. Nuôi Watchdog tránh reset h? th?ng */
        nrf_drv_wdt_channel_feed(m_channel_id);

        /* 5. Ch? s? ki?n (Ng? ti?t ki?m di?n) */
        (void)sd_app_evt_wait();
    }
}