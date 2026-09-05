#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"


/* =========================================================
 * CONFIGURACION DE HARDWARE
 * ========================================================= */

#define I2C_PORT        I2C_NUM_0

#define I2C_SDA_GPIO    GPIO_NUM_6   // D4
#define I2C_SCL_GPIO    GPIO_NUM_7   // D5

#define I2C_FREQ_HZ     100000

/* SA0 = GND -> 0x18
 * SA0 = VDD -> 0x19
 */
#define LIS2DH12_ADDR   0x19

/* INT1 del LIS2DH12 */
#define INT1_GPIO       GPIO_NUM_3


/* =========================================================
 * PARAMETROS DE DETECCION
 *
 * AJUSTA PRINCIPALMENTE ESTOS PARAMETROS
 * ========================================================= */


/*
 * ODR = 10 Hz
 * X, Y, Z habilitados
 */
#define MOTION_CTRL1            0x27


/*
 * HIGH-PASS FILTER PARA INT1
 *
 * CTRL_REG2:
 *
 * HPM  = 00   -> modo normal
 * HPCF = 00   -> corte ~0.2 Hz con ODR 10 Hz
 * HP_IA1 = 1  -> HPF aplicado a INT1
 *
 * Resultado: 0x01
 */
#define MOTION_HPF              0x01


/*
 * Umbral de movimiento.
 *
 * En +-2 g:
 * 1 paso ~= 16 mg
 *
 * 0x10 -> ~256 mg
 * 0x18 -> ~384 mg
 * 0x20 -> ~512 mg
 * 0x28 -> ~640 mg
 * 0x30 -> ~768 mg
 *
 * Con HPF ya no necesitas trabajar
 * cerca de 1 g para evitar la gravedad.
 */
#define MOTION_THRESHOLD        0x20


/*
 * Duracion minima.
 *
 * ODR = 10 Hz
 *
 * 0x00 -> practicamente inmediato
 * 0x01 -> ~100 ms
 * 0x02 -> ~200 ms
 * 0x03 -> ~300 ms
 */
#define MOTION_DURATION         0x00


/*
 * X+, Y+, Z+
 *
 * OR:
 * cualquiera de los 3 ejes
 * puede generar INT1.
 */
#define MOTION_INT1_CONFIG      0x2A


/*
 * Mantener INT1 activo
 * hasta leer INT1_SRC.
 */
#define MOTION_LATCH            0x08


/*
 * INT1 activo LOW:
 *
 * reposo      -> HIGH
 * interrupcion -> LOW
 */
#define MOTION_ACTIVE_LOW       0x02


/*
 * Generador IA1 -> pin INT1
 */
#define MOTION_ROUTE_INT1       0x40


/* =========================================================
 * REGISTROS LIS2DH12
 * ========================================================= */

#define REG_WHO_AM_I    0x0F

#define REG_CTRL1       0x20
#define REG_CTRL2       0x21
#define REG_CTRL3       0x22
#define REG_CTRL4       0x23
#define REG_CTRL5       0x24
#define REG_CTRL6       0x25

#define REG_REFERENCE   0x26

#define REG_INT1_CFG    0x30
#define REG_INT1_SRC    0x31
#define REG_INT1_THS    0x32
#define REG_INT1_DUR    0x33


/* =========================================================
 * VARIABLES
 * ========================================================= */

static const char *TAG = "LIS2DH12";

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;

static volatile bool int1_event = false;


/* =========================================================
 * ISR INT1
 * ========================================================= */

static void IRAM_ATTR int1_isr(void *arg)
{
    int1_event = true;
}


/* =========================================================
 * ESCRIBIR REGISTRO
 * ========================================================= */

static esp_err_t write_reg(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t data[2] = {
        reg,
        value
    };

    return i2c_master_transmit(
        dev_handle,
        data,
        sizeof(data),
        100
    );
}


/* =========================================================
 * LEER REGISTRO
 * ========================================================= */

static esp_err_t read_reg(
    uint8_t reg,
    uint8_t *value
)
{
    return i2c_master_transmit_receive(
        dev_handle,
        &reg,
        1,
        value,
        1,
        100
    );
}


/* =========================================================
 * INICIALIZAR I2C
 * ========================================================= */

static esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t bus_config = {

        .i2c_port = I2C_PORT,

        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,

        .clk_source = I2C_CLK_SRC_DEFAULT,

        .glitch_ignore_cnt = 7,

        .flags.enable_internal_pullup = true,
    };


    esp_err_t ret;


    ret = i2c_new_master_bus(
        &bus_config,
        &bus_handle
    );


    if (ret != ESP_OK)
    {
        return ret;
    }


    i2c_device_config_t dev_config = {

        .dev_addr_length =
            I2C_ADDR_BIT_LEN_7,

        .device_address =
            LIS2DH12_ADDR,

        .scl_speed_hz =
            I2C_FREQ_HZ,
    };


    return i2c_master_bus_add_device(
        bus_handle,
        &dev_config,
        &dev_handle
    );
}


/* =========================================================
 * CONFIGURAR GPIO INT1 ESP32
 * ========================================================= */

static esp_err_t int1_gpio_init(void)
{
    gpio_config_t io_conf = {

        .pin_bit_mask =
            1ULL << INT1_GPIO,

        .mode =
            GPIO_MODE_INPUT,

        .pull_up_en =
            GPIO_PULLUP_DISABLE,

        .pull_down_en =
            GPIO_PULLDOWN_DISABLE,

        /*
         * INT1 normalmente HIGH
         * y baja cuando ocurre evento.
         */
        .intr_type =
            GPIO_INTR_NEGEDGE
    };


    esp_err_t ret;


    ret = gpio_config(
        &io_conf
    );


    if (ret != ESP_OK)
    {
        return ret;
    }


    ret = gpio_install_isr_service(0);


    if (
        ret != ESP_OK &&
        ret != ESP_ERR_INVALID_STATE
    )
    {
        return ret;
    }


    return gpio_isr_handler_add(
        INT1_GPIO,
        int1_isr,
        NULL
    );
}


/* =========================================================
 * CONFIGURAR LIS2DH12
 * ========================================================= */

static esp_err_t lis2dh12_config_int1(void)
{
    esp_err_t ret;
    uint8_t dummy;


    /*
     * Desactivar temporalmente el generador
     * mientras configuramos.
     */
    ret = write_reg(
        REG_INT1_CFG,
        0x00
    );

    if (ret != ESP_OK)
        return ret;


    /* -----------------------------------------
     * 10 Hz + XYZ
     * ----------------------------------------- */

    ret = write_reg(
        REG_CTRL1,
        MOTION_CTRL1
    );

    if (ret != ESP_OK)
        return ret;


    /* -----------------------------------------
     * HIGH-PASS FILTER
     *
     * HP_IA1 = 1
     * ----------------------------------------- */

    ret = write_reg(
        REG_CTRL2,
        MOTION_HPF
    );

    if (ret != ESP_OK)
        return ret;


    /* -----------------------------------------
     * Escala +-2 g
     *
     * CTRL_REG4 = 0x00
     * ----------------------------------------- */

    ret = write_reg(
        REG_CTRL4,
        0x00
    );

    if (ret != ESP_OK)
        return ret;


    /* -----------------------------------------
     * IA1 -> INT1
     * ----------------------------------------- */

    ret = write_reg(
        REG_CTRL3,
        MOTION_ROUTE_INT1
    );

    if (ret != ESP_OK)
        return ret;


    /* -----------------------------------------
     * Latch INT1
     * ----------------------------------------- */

    ret = write_reg(
        REG_CTRL5,
        MOTION_LATCH
    );

    if (ret != ESP_OK)
        return ret;


    /* -----------------------------------------
     * INT1 activo LOW
     * ----------------------------------------- */

    ret = write_reg(
        REG_CTRL6,
        MOTION_ACTIVE_LOW
    );

    if (ret != ESP_OK)
        return ret;


    /* -----------------------------------------
     * Umbral
     * ----------------------------------------- */

    ret = write_reg(
        REG_INT1_THS,
        MOTION_THRESHOLD
    );

    if (ret != ESP_OK)
        return ret;


    /* -----------------------------------------
     * Duracion
     * ----------------------------------------- */

    ret = write_reg(
        REG_INT1_DUR,
        MOTION_DURATION
    );

    if (ret != ESP_OK)
        return ret;


    /*
     * Esperar algunas muestras antes
     * de inicializar la referencia HPF.
     */
    vTaskDelay(
        pdMS_TO_TICKS(500)
    );


    /* -----------------------------------------
     * REINICIAR REFERENCIA DEL HPF
     *
     * En HPM = 00, leer REFERENCE
     * reinicia el filtro tomando el
     * estado actual como referencia.
     * ----------------------------------------- */

    ret = read_reg(
        REG_REFERENCE,
        &dummy
    );

    if (ret != ESP_OK)
        return ret;


    /*
     * Esperar algunas muestras para
     * estabilizar el filtro.
     */
    vTaskDelay(
        pdMS_TO_TICKS(500)
    );


    /*
     * Limpiar posible evento anterior.
     */
    ret = read_reg(
        REG_INT1_SRC,
        &dummy
    );

    if (ret != ESP_OK)
        return ret;


    /* -----------------------------------------
     * ACTIVAR GENERADOR INT1 AL FINAL
     *
     * X+ OR Y+ OR Z+
     * ----------------------------------------- */

    ret = write_reg(
        REG_INT1_CFG,
        MOTION_INT1_CONFIG
    );


    return ret;
}


/* =========================================================
 * MOSTRAR REGISTRO
 * ========================================================= */

static void print_reg(
    const char *name,
    uint8_t reg
)
{
    uint8_t value = 0;


    esp_err_t ret = read_reg(
        reg,
        &value
    );


    if (ret == ESP_OK)
    {
        ESP_LOGI(
            TAG,
            "%-10s [0x%02X] = 0x%02X",
            name,
            reg,
            value
        );
    }
    else
    {
        ESP_LOGE(
            TAG,
            "%-10s ERROR",
            name
        );
    }
}


/* =========================================================
 * MOSTRAR CONFIGURACION
 * ========================================================= */

static void print_configuration(void)
{
    ESP_LOGI(
        TAG,
        "=============================="
    );

    ESP_LOGI(
        TAG,
        "CONFIGURACION LIS2DH12"
    );

    ESP_LOGI(
        TAG,
        "=============================="
    );


    print_reg(
        "CTRL1",
        REG_CTRL1
    );

    print_reg(
        "CTRL2",
        REG_CTRL2
    );

    print_reg(
        "CTRL3",
        REG_CTRL3
    );

    print_reg(
        "CTRL4",
        REG_CTRL4
    );

    print_reg(
        "CTRL5",
        REG_CTRL5
    );

    print_reg(
        "CTRL6",
        REG_CTRL6
    );

    print_reg(
        "INT1_CFG",
        REG_INT1_CFG
    );

    print_reg(
        "INT1_THS",
        REG_INT1_THS
    );

    print_reg(
        "INT1_DUR",
        REG_INT1_DUR
    );


    ESP_LOGI(
        TAG,
        "=============================="
    );
}


/* =========================================================
 * APP MAIN
 * ========================================================= */

void app_main(void)
{
    esp_err_t ret;

    uint8_t who_am_i = 0;
    uint8_t int1_src = 0;


    /* =====================================================
     * I2C
     * ===================================================== */

    ESP_LOGI(
        TAG,
        "Inicializando I2C..."
    );


    ret = i2c_init();


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Error I2C: %s",
            esp_err_to_name(ret)
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "I2C OK"
    );


    /* =====================================================
     * WHO_AM_I
     * ===================================================== */

    ret = read_reg(
        REG_WHO_AM_I,
        &who_am_i
    );


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "No responde WHO_AM_I"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "WHO_AM_I = 0x%02X",
        who_am_i
    );


    if (who_am_i != 0x33)
    {
        ESP_LOGE(
            TAG,
            "Sensor inesperado"
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "LIS2DH12 detectado"
    );


    /* =====================================================
     * GPIO INT1
     * ===================================================== */

    ret = int1_gpio_init();


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Error GPIO INT1"
        );

        return;
    }


    /* =====================================================
     * CONFIGURAR SENSOR
     * ===================================================== */

    ESP_LOGI(
        TAG,
        "Configurando HPF..."
    );


    ret = lis2dh12_config_int1();


    if (ret != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Error configurando LIS2DH12: %s",
            esp_err_to_name(ret)
        );

        return;
    }


    ESP_LOGI(
        TAG,
        "LIS2DH12 configurado"
    );


    /* =====================================================
     * MOSTRAR REGISTROS
     * ===================================================== */

    print_configuration();


    ESP_LOGI(
        TAG,
        "HPF habilitado para INT1"
    );

    ESP_LOGI(
        TAG,
        "INT1 reposo = HIGH"
    );

    ESP_LOGI(
        TAG,
        "Movimiento = LOW"
    );


    /* =====================================================
     * LOOP
     * ===================================================== */

    while (1)
    {
        int gpio_level =
            gpio_get_level(
                INT1_GPIO
            );


        /*
         * ISR detecto HIGH -> LOW
         */
        if (int1_event)
        {
            int1_event = false;


            ESP_LOGW(
                TAG,
                ">>> MOVIMIENTO DETECTADO <<<"
            );
        }


        /*
         * Leer fuente.
         *
         * Al estar LATCH habilitado,
         * esta lectura libera INT1.
         */
        ret = read_reg(
            REG_INT1_SRC,
            &int1_src
        );


        if (ret == ESP_OK)
        {
            ESP_LOGI(
                TAG,
                "INT1=%d | SRC=0x%02X",
                gpio_level,
                int1_src
            );
        }


        vTaskDelay(
            pdMS_TO_TICKS(500)
        );
    }
}