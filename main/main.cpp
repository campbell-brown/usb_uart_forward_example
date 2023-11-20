// ESP-IDF includes ------------------------------------------------------------
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"


// System includes -------------------------------------------------------------
#include <algorithm>


// Definitions -----------------------------------------------------------------
#define PIN_TX                      (4)
#define PIN_RX                      (10)

#define TASK_STACK_SIZE             (2048)
#define TASK_PARAMETERS             (NULL)
#define FORWARD_TASK_PRIORITY       (10)
#define DELAY_SHORT                 (20 / portTICK_PERIOD_MS)
#define MAX_FORWARD_CHUNK_LEN       (1024)

#define UART_BUF_SIZE               (1024)
#define UART_TIMEOUT                (300 / portTICK_PERIOD_MS)


// Static variables ------------------------------------------------------------
static TaskHandle_t task_handle  = NULL;
static uint8_t forward_buffer[MAX_FORWARD_CHUNK_LEN] = { 0 };


// Static function prototypes --------------------------------------------------
static void uart_init(void);
static size_t uart_num_bytes_available(void);
static bool uart_are_bytes_available(void);
static size_t uart_read_all(uint8_t *buffer, size_t max_length);
static size_t uart_write(const uint8_t *buffer, size_t length);
static void uart_send(void);

static void usb_init(void);
static bool usb_are_bytes_available(void);
static size_t usb_read_all(uint8_t *buffer, size_t max_length);
static size_t usb_write(const uint8_t *buffer, size_t length);
static void usb_send(void);

static void forward_task(void *args);
static void forward_usb_to_uart(void);
static void forward_uart_to_usb(void);


// Public functions ------------------------------------------------------------
extern "C" void app_main(void)
{
    uart_init();
    usb_init();
    xTaskCreate(forward_task,
                "forward_task",
                TASK_STACK_SIZE,
                TASK_PARAMETERS,
                FORWARD_TASK_PRIORITY,
                &task_handle);

    // Short delay for system to settle
    vTaskDelay(DELAY_SHORT);
}


// Private functions -----------------------------------------------------------
static void forward_task(void *args)
{
    while (true)
    {
        forward_usb_to_uart();
        forward_uart_to_usb();
        taskYIELD();
    }
}


static void uart_init(void)
{
    const uart_config_t config =
    {
        .baud_rate           = (9600),
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_EVEN,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = (0),
        .source_clk          = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, PIN_RX, PIN_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, UART_BUF_SIZE, 0, 0, NULL, 0));
}


static void usb_init(void) { }


static void forward_usb_to_uart(void)
{
    bool are_bytes_available = usb_are_bytes_available();
    if (are_bytes_available)
    {
        size_t bytes_read = usb_read_all(forward_buffer, MAX_FORWARD_CHUNK_LEN);
        size_t bytes_written = 0;
        while (bytes_written < bytes_read)
        {
            size_t bytes_left = bytes_read - bytes_written;
            bytes_written += uart_write(forward_buffer + bytes_written, bytes_left);
            uart_send();
        }
        uart_send();
    }
}


static void forward_uart_to_usb(void)
{
    bool are_bytes_available = uart_are_bytes_available();
    if (are_bytes_available)
    {
        size_t bytes_read = uart_read_all(forward_buffer, MAX_FORWARD_CHUNK_LEN);
        size_t bytes_written = 0;
        while (bytes_written < bytes_read)
        {
            size_t bytes_left = bytes_read - bytes_written;
            bytes_written += usb_write(forward_buffer + bytes_written, bytes_left);
            usb_send();
        }
        usb_send();
    }
}


static size_t uart_num_bytes_available(void)
{
    size_t available_bytes = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_1, &available_bytes));
    return available_bytes;
}


static bool uart_are_bytes_available(void)
{
    return uart_num_bytes_available() > 0;
}


static size_t uart_read_all(uint8_t *buffer, size_t max_length)
{
    size_t length = std::min(max_length, uart_num_bytes_available());
    return (size_t)uart_read_bytes(UART_NUM_1, buffer, (uint32_t)length, UART_TIMEOUT);
}


static size_t uart_write(const uint8_t *buffer, size_t length)
{
    return (size_t)uart_write_bytes(UART_NUM_1, buffer, length);
}


static void uart_send(void)
{
    while (uart_wait_tx_done(UART_NUM_1, UART_TIMEOUT) == ESP_ERR_TIMEOUT);
}


static bool usb_are_bytes_available(void)
{
    return (bool)usb_serial_jtag_ll_rxfifo_data_available();
}


static size_t usb_read_all(uint8_t *buffer, size_t max_length)
{
    size_t length = std::min(max_length, (size_t)USB_SERIAL_JTAG_PACKET_SZ_BYTES);
    return (size_t)usb_serial_jtag_ll_read_rxfifo(buffer, length); // Reads until no more bytes are available
}


static size_t usb_write(const uint8_t *buffer, size_t length)
{
    int can_write = usb_serial_jtag_ll_txfifo_writable();
    while (!can_write)
    {
        vTaskDelay(1);
        can_write = usb_serial_jtag_ll_txfifo_writable();
    }
    return (size_t)usb_serial_jtag_ll_write_txfifo(buffer, length);
}


static void usb_send(void)
{
    usb_serial_jtag_ll_txfifo_flush();
}
