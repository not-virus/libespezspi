/**
 * libespezspi.c
 * 
 * Written by Cameron Krueger
 * 8 August 2021
 * 
 * Implementation file for libespezspi, an attempt to create a simpler and
 * more manageable API for 8-bit SPI applications using the ESP8266 RTOS SDK
 * spi driver
 */
#include "libespezspi.h"

#define EZSPI_PIN_MISO CONFIG_EZSPI_PIN_MISO
#define EZSPI_PIN_MOSI CONFIG_EZSPI_PIN_MOSI
#define EZSPI_PIN_CLK CONFIG_EZSPI_PIN_CLK

static const char* TAG = "libespezspi";

// Local utility functions
void conv_arr_uint8_uint32_right(const uint8_t* s, uint32_t len, uint32_t* d);
void conv_arr_uint8_uint32_left(const uint8_t* s, uint32_t len, uint32_t* d);
void reverse_uint8_arr(uint8_t* arr, uint32_t len);
void reverse_uint16_arr(uint16_t* arr, uint32_t len);
void reverse_uint32_arr(uint32_t* arr, uint32_t len);
uint32_t reverse_uint32_byte_order(uint32_t s);

/**
 * Initializes the ESP8266 hardware SPI interface with the following settings:
 *  - //FIXME list settings
 */
esp_err_t init_spi() {

    ESP_LOGI(TAG, "init spi");
    spi_config_t spi_config;
    spi_config.interface.val = SPI_DEFAULT_INTERFACE;
    //spi_config.interface.byte_tx_order = SPI_BYTE_ORDER_MSB_FIRST;    // is borked
    spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
    spi_config.mode = SPI_MASTER_MODE;
    spi_config.clk_div = SPI_2MHz_DIV;     // Math is okay, but may be too fast
    spi_config.event_cb = NULL;
    esp_err_t tmp = spi_init(HSPI_HOST, &spi_config);

    if (tmp == ESP_OK) {
        ESP_LOGI(TAG, "spi init successful");
    } else {
        ESP_LOGE(TAG, "spi init failed");
    }

    return tmp;
}

/**
 * Sends a 1-byte command followed by len bytes of data over the SPI interface
 * 
 * NOTICE: init_spi() must be called once to initialize the SPI interface before
 * this function can be used!
 * 
 * @param cmd a 1-byte command for the SPI device
 * @param data an array of bytes of data for the SPI device
 * @param len the length of the data array
 * @return ESP_OK if transmission successful, else ESP_FAIL
 */
esp_err_t spi_send_command_with_data(uint8_t cmd, const uint8_t* data,
                                     uint32_t len) {

    ESP_LOGD(TAG, "allocate memory for spi_trans_t and clear");
    // Allocate memory for the spi transmission parameter structure
    spi_trans_t trans;
    memset(&trans, 0x0, sizeof(trans));
    trans.bits.val = 0; // Clear all bits

    ESP_LOGD(TAG, "set spi command");
    // Set command to write to peripheral
    uint16_t cmd16 = cmd;
    // 8 bits for command
    trans.bits.cmd = 8;
    trans.cmd = &cmd16;

    ESP_LOGD(TAG, "set address value");
    // When writing a command, the address isn't used
    trans.bits.addr = 0;
    trans.addr = NULL;

    ESP_LOGD(TAG, "allocate memory for uint32_t array");
    // Convert data to uint32_t array
    uint32_t* new_data = (uint32_t*) malloc(len * sizeof(*new_data));

    ESP_LOGD(TAG, "copy uint8_t data into new uint32_t array");
    conv_arr_uint8_uint32_left(data, len, new_data);

    printf("Before reversal:\n");
    for (size_t i = 0; i < 2; i++) {
        printf("0x%08x\n", (uint32_t) *(new_data + i));
    }

    // FIXME Possibly hardcode for speed?
    uint8_t sizeof_s = sizeof(*data);
    uint8_t sizeof_d = sizeof(*new_data);

    // Get ratio of destination data width to source data width
    uint32_t type_ratio = sizeof_d / sizeof_s;
    
    // Size of destination array in bytes
    uint32_t new_len = ceil(len / (double) type_ratio);

    // Due to a bug in the ESP8266 RTOS SDK, we have to reverse the byte
    //  order of the transmission data ourselves. Using
    //  SPI_BYTE_ORDER_MSB_FIRST in SPI configuration causes every 4th byte
    //  in trans.miso to be sent as 0xFF
    ESP_LOGD(TAG, "reverse data array");
    for (int i = 0; i < new_len; i++) {
        new_data[i] = reverse_uint32_byte_order(new_data[i]);
    }
    //reverse_uint32_arr(new_data, new_len);

    printf("After reversal:\n");
    for (size_t i = 0; i < new_len; i++) {
        printf("0x%08x\n", (uint32_t) *(new_data + i));
    }

    ESP_LOGD(TAG, "set length of data and data in spi_trans_t");
    // Convert len to reflect bits, not bytes
    len = (uint32_t) 8 * len;
    trans.bits.mosi = len;
    trans.mosi = new_data;

    ESP_LOGD(TAG, "transmit spi data to device");
    esp_err_t res = spi_trans(HSPI_HOST, &trans);

    ESP_LOGD(TAG, "free temporary uint32_t pointer");
    free(new_data);

    if (res == ESP_OK) {
        ESP_LOGD(TAG, "transmission successful");
    } else {
        ESP_LOGD(TAG, "spi transmission failed");
    }

    return res;
}

/**
 * Sends a 1-byte command over the SPI interface.
 * 
 * NOTICE: init_spi() must be called once to initialize the SPI interface before
 * this function can be used!
 * 
 * @param cmd a 1-byte command for the SPI device
 * @return ESP_OK if transmission successful, else ESP_FAIL
 */
esp_err_t spi_send_command(uint8_t cmd) {

    ESP_LOGD(TAG, "allocate memory for spi_trans_t and clear");
    // Allocate memory for the spi transmission parameter structure
    spi_trans_t trans;
    memset(&trans, 0x0, sizeof(trans));
    trans.bits.val = 0; // Clear all bits

    ESP_LOGD(TAG, "set spi command");
    // Set command to write to peripheral
    uint16_t cmd16 = cmd;
    // 8 bits for command
    trans.bits.cmd = 8;
    trans.cmd = &cmd16;

    ESP_LOGD(TAG, "set address value (NULL)");
    // When writing a command, the address isn't used
    trans.bits.addr = 0;
    trans.addr = NULL;

    ESP_LOGD(TAG, "set data value (NULL)");
    // When writing a command without data, the data isn't used
    trans.bits.mosi = 0;
    trans.mosi = NULL;

    ESP_LOGD(TAG, "transmit spi data to device");
    esp_err_t res = spi_trans(HSPI_HOST, &trans);

    if (res == ESP_OK) {
        ESP_LOGD(TAG, "transmission successful");
    } else {
        ESP_LOGD(TAG, "spi transmission failed");
    }

    return res;
}

/**
 * @brief Writes raw bytes to SPI device
 * 
 * @param data an array of bytes of data for the SPI device
 * @param len the number of bytes to be sent
 */
esp_err_t spi_send_raw(const uint8_t* data, uint32_t len) {

    ESP_LOGI(TAG, "allocate memory for spi_trans_t and clear");
    // Allocate memory for the spi transmission parameter structure
    spi_trans_t trans;
    memset(&trans, 0x0, sizeof(trans));
    trans.bits.val = 0; // Clear all bits

    ESP_LOGD(TAG, "set spi command");
    // When writing raw data, the command isn't used
    trans.bits.cmd = 0;
    trans.cmd = NULL;

    ESP_LOGD(TAG, "set address value");
    // When writing a command, the address isn't used
    trans.bits.addr = 0;
    trans.addr = NULL;

    ESP_LOGD(TAG, "allocate memory for uint32_t array");
    // Convert data to uint32_t array
    uint32_t* new_data = (uint32_t*) malloc(len * sizeof(*new_data));

    ESP_LOGD(TAG, "copy uint8_t data into new uint32_t array");
    conv_arr_uint8_uint32_left(data, len, new_data);

    printf("Before reversal:\n");
    for (size_t i = 0; i < 2; i++) {
        printf("0x%08x\n", *(new_data + i));
    }

    // Due to a bug in the ESP8266 RTOS SDK, we have to reverse this
    //  ourselves. Using SPI_BYTE_ORDER_MSB_FIRST in SPI configuration
    //  causes every 4th byte in trans.miso to be sent as 0xFF
    ESP_LOGD(TAG, "reverse data array");
    reverse_uint32_arr(new_data, len);

    ESP_LOGD(TAG, "set length of data and data in spi_trans_t");
    // Convert len to reflect bits, not bytes
    len = (uint32_t) 8 * len;
    trans.bits.mosi = len;
    trans.mosi = new_data;

    printf("After reversal:\n");
    for (size_t i = 0; i < trans.bits.mosi / 8; i++) {
        printf("0x%08x\n", *(trans.mosi + i));
    }

    ESP_LOGD(TAG, "transmit spi data to device");
    esp_err_t res = spi_trans(HSPI_HOST, &trans);

    ESP_LOGD(TAG, "free temporary uint32_t pointer");
    free(new_data);

    if (res == ESP_OK) {
        ESP_LOGD(TAG, "transmission successful");
    } else {
        ESP_LOGD(TAG, "spi transmission failed");
    }

    return res;
}

/**
 * Reads len bytes from the SPI interface and places them in buf
 * 
 * @param len the number of bytes to read from the device
 * @param buf a buffer of length len
 */
void spi_read(unsigned char* buf, uint16_t len) {
    // FIXME
    return;
}


/**
 * Reverses a uint8_t array
 * 
 * @param arr a uint8_t array
 * @param len the number of uint8_t elements (bytes) in arr
 */
void reverse_uint8_arr(uint8_t* arr, uint32_t len) {
    uint32_t start = 0;
    uint32_t end = len - 1;
    while (start < end) {
        uint8_t tmp = *(arr + start);
        *(arr + start) = *(arr + end);
        *(arr + end) = tmp;
        start++;
        end--;
    }
}

/**
 * Reverses a uint16_t array
 * 
 * @param arr a uint16_t array
 * @param len the number of uint16_t elements in arr
 */
void reverse_uint16_arr(uint16_t* arr, uint32_t len) {
    uint32_t start = 0;
    uint32_t end = len - 1;
    while (start < end) {
        uint16_t tmp = *(arr + start);
        *(arr + start) = *(arr + end);
        *(arr + end) = tmp;
        start++;
        end--;
    }
}

/**
 * Reverses a uint32_t array
 * 
 * @param arr a uint32_t array
 * @param len the number of uint32_t elements in arr
 */
void reverse_uint32_arr(uint32_t* arr, uint32_t len) {
    uint32_t start = 0;
    uint32_t end = len - 1;
    while (start < end) {
        uint32_t tmp = *(arr + start);
        *(arr + start) = *(arr + end);
        *(arr + end) = tmp;
        start++;
        end--;
    }
}

/**
 * Reverses the byte order of a uint32_t
 * 
 * @param data the uint32_t to reverse
 */
uint32_t reverse_uint32_byte_order(uint32_t s) {
    uint8_t num_bytes = sizeof(s) / sizeof(uint8_t) - 1;
    uint32_t reversed = 0;
    uint8_t cur_byte;

    printf("0x%02x\n", s);

    for(size_t i = 0; i <= num_bytes; i++) {
        cur_byte = (s>>(i * 8));
        reversed |= (cur_byte<<((num_bytes - i) * 8));
    }

    return reversed;
}

/**
 * Converts a uint8_t array to a uint32_t array and aligns the source data to
 * the right of the destination array
 *   i.e. {0x01, 0x02, 0x03} -> {0x00010203}
 *
 * NOTICE: This function uses pointers and pointer/offset arithmetic. It is
 * absolutely not safe. However many elements (in this case bytes) of the source
 * array are specified by len to be copied is how many this function will
 * attempt to copy. If len is larger than the actual size of s, undefined
 * behavior and/or a segfault is/are absolutely possible. USE WITH CARE.
 * 
 * @param s source array
 * @param len len of source array
 * @param d destination array
 * @return ESP_OK
 */
void conv_arr_uint8_uint32_right(const uint8_t* s, uint32_t len, uint32_t* d) {

    // FIXME Possibly hardcode for speed?
    uint8_t sizeof_s = sizeof(*s);
    uint8_t sizeof_d = sizeof(*d);

    // Get ratio of destination data width to source data width
    uint32_t type_ratio = sizeof_d / sizeof_s;

    // Size of destination array in bytes
    size_t dest_size = (size_t) sizeof_d * ceil(len / (double) type_ratio);

    // Clear all memory in array range
    memset(d, 0x0, dest_size);

    // Offset determines number of empty leading bytes for right alignment
    size_t offset = dest_size - len;
    
    // Copy source array into destination array
    size_t src_index;
    for (size_t dest_index = 0; dest_index < len / (double) type_ratio;
            dest_index++) {
        size_t m = dest_index * type_ratio;
        if (dest_index == 0) {
            src_index = offset;
        } else {
            src_index = 0;
        }
        for (; src_index < type_ratio; src_index++) {
            *(d + dest_index)|= (*(s + ((src_index - offset) + m))
                << (((type_ratio - src_index) - 1) * sizeof_s * 8));
        }
    }
}

/**
 * Converts a uint8_t array s to a uint32_t array d and left-aligns the
 * data in d
 *   i.e. {0x01, 0x02, 0x03} -> {0x01020300} 
 * 
 * NOTICE: This function uses pointers and pointer/offset arithmetic. It is
 * absolutely not safe. However many elements (in this case bytes) of the source
 * array are specified by len to be copied is how many this function will
 * attempt to copy. If len is larger than the actual size of s, undefined
 * behavior and/or a segfault is/are absolutely possible. USE WITH CARE.
 * 
 * @param s source array
 * @param len len of source array
 * @param d destination array
 * @return ESP_OK
 */
void conv_arr_uint8_uint32_left(const uint8_t* s, uint32_t len, uint32_t* d) {

    // FIXME Possibly hardcode for speed?
    uint8_t sizeof_s = sizeof(*s);
    uint8_t sizeof_d = sizeof(*d);

    // Get ratio of destination data width to source data width
    uint32_t type_ratio = sizeof_d / sizeof_s;

    // Size of destination array in bytes
    size_t dest_size = (size_t) sizeof_d * ceil(len / (double) type_ratio);

    // Clear all memory in array range
    memset(d, 0x00, dest_size);

    // Copy source array into destination array
    for (size_t dest_index = 0; dest_index < len / (double) type_ratio;
            dest_index++) {
        size_t m = dest_index * type_ratio;
        for (size_t src_index = 0; src_index < type_ratio; src_index++) {
            if (m + src_index >= len) {
                break;
            }
            *(d + dest_index) |= (*(s + (src_index + m))
                << (((type_ratio - src_index) - 1) * sizeof_s * 8));
        }
    }
}