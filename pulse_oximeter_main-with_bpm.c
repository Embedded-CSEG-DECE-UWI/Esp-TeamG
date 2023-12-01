#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"

#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "C:\Users\PCUser\esp\ESP8266_RTOS_SDK\components\freertos\include\freertos\semphr.h"

static const char *TAG = "main";

static SemaphoreHandle_t mutex_pin;
TaskHandle_t task_adc,task_calc;
static uint16_t adc_data_red[300], adc_data_ir[300],thresh,thresl;
static uint8_t red_peaks[20],ir_peaks[20];
TickType_t adc_data_time[300];
int interval = 300;
int bpm;
#define I2C_EXAMPLE_MASTER_SCL_IO           0                /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           2               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define ADC_ADDR                            0x48             /*!< slave address for MPU6050 sensor */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

/**
 * Define the mpu6050 register address:
 */
#define CONVER          0x00
#define CONFIG          0x01

/**
 * @brief i2c master initialization
 */
static void dly(int ms)
{
    TickType_t bstart = xTaskGetTickCount();
    TickType_t now;
    while (((now = xTaskGetTickCount()) - bstart) < (ms/portTICK_PERIOD_MS))
    {
        /* burn time here */
    }
}
static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 30; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return ESP_OK;
}

/**
 * @brief test code to write mpu6050
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_example_master_mpu6050_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief test code to read mpu6050
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
static esp_err_t i2c_example_master_mpu6050_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADC_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADC_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t adc_low_power(i2c_port_t i2c_num)
{
    uint8_t cmd_data[2];
    //vTaskDelay(100 / portTICK_RATE_MS); check if system no work
    cmd_data[0] = 0b01000011;
    cmd_data[1] = 0b10000011;
    ESP_ERROR_CHECK(i2c_example_master_mpu6050_write(i2c_num, CONFIG, cmd_data, 2));
    return ESP_OK;
}

static esp_err_t adc_channel_sel(i2c_port_t i2c_num,int channel)
{
    uint8_t cmd_data[2];
    //vTaskDelay(100 / portTICK_RATE_MS); check if system no work
    if (channel==0)
    {
        cmd_data[0] = 0b01000010;
    }
    else if (channel==1)
    {
        cmd_data[0] = 0b01010010;
    }
    cmd_data[1] = 0b10000011;
        
    ESP_ERROR_CHECK(i2c_example_master_mpu6050_write(i2c_num, CONFIG, cmd_data, 2));
    return ESP_OK;
}

static void adc_sample(void *arg)
{
    uint8_t sensor_data[2];
    int ret,i=0;
    memset(sensor_data, 0, 2);
    TickType_t bstart = xTaskGetTickCount();
    TickType_t now;
    while ((i<300) && ((now = xTaskGetTickCount()) - bstart) < (3000/portTICK_PERIOD_MS)) {
        adc_channel_sel(I2C_EXAMPLE_MASTER_NUM,0);
        ret = i2c_example_master_mpu6050_read(I2C_EXAMPLE_MASTER_NUM, CONVER, sensor_data, 2);
        if (ret == ESP_OK) { 
            adc_data_red[i]=(uint16_t)(sensor_data[0]<<8)|sensor_data[1];
            adc_data_time[i] = xTaskGetTickCount();
        } else {
            ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
        }
        adc_channel_sel(I2C_EXAMPLE_MASTER_NUM,1);
        ret = i2c_example_master_mpu6050_read(I2C_EXAMPLE_MASTER_NUM, CONVER, sensor_data, 2);
        if (ret == ESP_OK) { 
            adc_data_ir[i]=(uint16_t)(sensor_data[0]<<8)|sensor_data[1];
        } else {
            ESP_LOGE(TAG, "No ack, sensor not connected...skip...\n");
        }
        i++;
        dly(10);
        adc_low_power(I2C_EXAMPLE_MASTER_NUM);
        vTaskResume(task_calc);
        vTaskDelay(interval*1000 / portTICK_PERIOD_MS);
    }

    i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}
void peak_find ()
{
    
    memset(red_peaks,300,20);
    memset(ir_peaks,300,20);
    int j=0, high=0;
    while(i<300)
    {
        while(adc_data_red[i]>thresh)
        {
            if(adc_data_red[i]=>high)
            {
                high=adc_data_red[i];
                red_peaks[j]=i;
            }
            i++;

        }
        j++;
        i++;
        high=0;
    }
    i,j=0;
    while(i<300)
    {
        while(adc_data_ir[i]>thresh)
        {
            if(adc_data_ir[i]=>high)
            {
                high=adc_data_ir[i];
                ir_peaks[j]=i;
            }
            i++;

        }
        j++;
        i++;
        high=0;
    }
}

void calculation ()
{
    peak_find();
    double sum=0;
    int i=1;
    while (red_peaks[i] < 300 && i < 20)
    {
        sum+=double((adc_data_time[red_peaks[i]]-adc_data_time[red_peaks[i-1]]));
        i++;
    }
    avg=sum/i;
    bpm=60000/(avg*portTICK_PERIOD_MS);
}
void app_main(void)
{
    int ret;
    ret=i2c_master_init();
    if (ret == ESP_OK) {
            printf("successful initalise\n");
        } else {
            printf("failed initalise\n");
        } 
    xTaskCreate(adc_sample, "Task_ADC_Sample", 2048, NULL, 12, &task_adc);
}