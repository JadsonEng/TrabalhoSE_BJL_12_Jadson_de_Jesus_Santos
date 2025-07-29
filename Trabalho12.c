#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/bootrom.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/rtc.h"
#include "ssd1306.h"
#include "font.h"

// Includes para SD Card
#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"

// Definições de GPIO
#define BOTAO_A 5
#define BOTAO_B 6
#define LED_PIN 7
#define BUZZER_PIN 21
#define RED_PIN 13
#define BLUE_PIN 12
#define GREEN_PIN 11

// I2C para MPU6050
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1

// I2C para Display
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define ENDERECO_DISP 0x3C
#define DISP_W 128
#define DISP_H 64

// MPU6050
static int mpu_addr = 0x68;

// Estados do sistema
typedef enum {
    STATE_INITIALIZING,
    STATE_READY,
    STATE_CAPTURING,
    STATE_SD_ACCESS,
    STATE_ERROR
} system_state_t;

// Declarações de funções
void set_rgb_color(bool red, bool green, bool blue);
void led_yellow();
void led_green();
void led_red();
void led_blue();
void led_purple();
void led_off();
void buzzer_beep(uint16_t frequency, uint32_t duration_ms);
void buzzer_single_beep();
void buzzer_double_beep();
void update_display(const char* status, const char* info);
static void mpu6050_reset();
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]);
static sd_card_t *sd_get_by_name(const char *const name);
static FATFS *sd_get_fs_by_name(const char *name);
bool mount_sd_card();
bool unmount_sd_card();
bool create_csv_file();
bool create_metadata_file();
bool write_sensor_data(uint32_t sample_num, int16_t accel[3], int16_t gyro[3]);
void gpio_irq_handler(uint gpio, uint32_t events);
void process_single_sample();
void system_init();
void generate_filename();

// Variáveis globais
static volatile system_state_t current_state = STATE_INITIALIZING;
static volatile bool capture_active = false;
static volatile bool sd_mounted = false;
static volatile bool button_a_pressed = false;
static volatile bool button_b_pressed = false;
static uint32_t sample_count = 0;
static char csv_filename[32];
static ssd1306_t ssd;

// Função para gerar nome único do arquivo com timestamp
void generate_filename() {
    static uint16_t file_counter = 1;
    snprintf(csv_filename, sizeof(csv_filename), "data_%03d.csv", file_counter++);
}

// Funções para controle de LED RGB
void set_rgb_color(bool red, bool green, bool blue) {
    gpio_put(RED_PIN, red);
    gpio_put(GREEN_PIN, green);
    gpio_put(BLUE_PIN, blue);
}

void led_yellow() { set_rgb_color(true, true, false); }
void led_green() { set_rgb_color(false, true, false); }
void led_red() { set_rgb_color(true, false, false); }
void led_blue() { set_rgb_color(false, false, true); }
void led_purple() { set_rgb_color(true, false, true); }
void led_off() { set_rgb_color(false, false, false); }

// Função para buzzer
void buzzer_beep(uint16_t frequency, uint32_t duration_ms) {
    // Configurar PWM para o buzzer
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    
    // Configurar frequência
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125000000.0f / (frequency * 4096));
    pwm_init(slice_num, &config, true);
    pwm_set_gpio_level(BUZZER_PIN, 2048); // 50% duty cycle
    
    sleep_ms(duration_ms);
    
    pwm_set_gpio_level(BUZZER_PIN, 0); // Desligar
}

void buzzer_single_beep() {
    buzzer_beep(1000, 300);
}

void buzzer_double_beep() {
    buzzer_beep(1000, 200);
    sleep_ms(100);
    buzzer_beep(1000, 200);
}

// Função para atualizar display
void update_display(const char* status, const char* info) {
    ssd1306_fill(&ssd, false);
    
    // Título
    ssd1306_draw_string(&ssd, "STATUS", 0, 0);
    ssd1306_draw_string(&ssd, sd_mounted ? "SD: OK" : "SD: N/A", 60, 0);
    ssd1306_line(&ssd, 0, 10, DISP_W-1, 10, true);
    
    // Status
    char status_line[32];
    snprintf(status_line, sizeof(status_line), "%s", status);
    ssd1306_draw_string(&ssd, status_line, 0, 15);
    
    // Informação adicional
    if (info) {
        ssd1306_draw_string(&ssd, info, 0, 25);
    }
    
    // Contador de amostras se capturando
    if (capture_active) {
        char sample_line[32];
        snprintf(sample_line, sizeof(sample_line), "Amostras: %lu", sample_count);
        ssd1306_draw_string(&ssd, sample_line, 0, 50);
    }
    
    // Status do SD
    ssd1306_draw_string(&ssd, sd_mounted ? "SD: OK" : "SD: N/A", 60, 0);
    
    // Controles
    //ssd1306_draw_string(&ssd, "A:Iniciar B:SD", 0, 55);
    
    ssd1306_send_data(&ssd);
}

// Funções MPU6050
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT, mpu_addr, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, mpu_addr, buf, 2, false);
    sleep_ms(10);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[6];
    uint8_t val = 0x3B;
    
    // Ler acelerômetro
    i2c_write_blocking(I2C_PORT, mpu_addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, mpu_addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];

    // Ler giroscópio
    val = 0x43;
    i2c_write_blocking(I2C_PORT, mpu_addr, &val, 1, true);
    i2c_read_blocking(I2C_PORT, mpu_addr, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
}

// Funções para SD Card
static sd_card_t *sd_get_by_name(const char *const name) {
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    return NULL;
}

static FATFS *sd_get_fs_by_name(const char *name) {
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    return NULL;
}

bool mount_sd_card() {
    current_state = STATE_SD_ACCESS;
    led_blue();
    update_display("Montando SD", "Aguarde...");
    
    const char *drive_name = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(drive_name);
    if (!p_fs) return false;
    
    FRESULT fr = f_mount(p_fs, drive_name, 1);
    if (FR_OK != fr) {
        current_state = STATE_ERROR;
        led_purple();
        update_display("ERRO SD", "Falha ao montar");
        return false;
    }
    
    sd_card_t *pSD = sd_get_by_name(drive_name);
    if (pSD) pSD->mounted = true;
    sd_mounted = true;
    
    return true;
}

bool unmount_sd_card() {
    current_state = STATE_SD_ACCESS;
    led_blue();
    update_display("Desmontando", "Aguarde...");
    
    const char *drive_name = sd_get_by_num(0)->pcName;
    FRESULT fr = f_unmount(drive_name);
    if (FR_OK != fr) return false;
    
    sd_card_t *pSD = sd_get_by_name(drive_name);
    if (pSD) {
        pSD->mounted = false;
        pSD->m_Status |= STA_NOINIT;
    }
    sd_mounted = false;
    
    return true;
}

bool create_csv_file() {
    FIL file;
    FRESULT res = f_open(&file, csv_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) return false;
    
    // Escrever cabeçalho CSV com separadores claros e nomes descritivos
    const char* header = "Amostra,Accel_X_LSB,Accel_Y_LSB,Accel_Z_LSB,Gyro_X_LSB,Gyro_Y_LSB,Gyro_Z_LSB,Accel_X_g,Accel_Y_g,Accel_Z_g,Gyro_X_dps,Gyro_Y_dps,Gyro_Z_dps,Timestamp_ms\r\n";
    UINT bw;
    res = f_write(&file, header, strlen(header), &bw);
    f_close(&file);
    
    // Criar também um arquivo de metadados para facilitar análise
    create_metadata_file();
    
    return (res == FR_OK);
}

// Função adicional para criar arquivo de metadados
bool create_metadata_file() {
    char metadata_filename[32];
    strcpy(metadata_filename, "metadata_");
    strcat(metadata_filename, csv_filename);
    
    FIL file;
    FRESULT res = f_open(&file, metadata_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (res != FR_OK) return false;
    
    const char* metadata = 
        "# Metadados do Dataset MPU6050\r\n"
        "# ===============================\r\n"
        "Sensor: MPU6050\r\n"
        "Taxa_Amostragem: 20Hz (50ms)\r\n"
        "Acelerometro_Range: ±2g\r\n"
        "Acelerometro_Sensibilidade: 16384 LSB/g\r\n"
        "Giroscopio_Range: ±250°/s\r\n"
        "Giroscopio_Sensibilidade: 131 LSB/°/s\r\n"
        "# ===============================\r\n"
        "# Colunas do CSV:\r\n"
        "# Amostra: Número sequencial da amostra\r\n"
        "# Accel_X/Y/Z_LSB: Valores brutos do acelerômetro\r\n"
        "# Gyro_X/Y/Z_LSB: Valores brutos do giroscópio\r\n"
        "# Accel_X/Y/Z_g: Aceleração em unidades g\r\n"
        "# Gyro_X/Y/Z_dps: Velocidade angular em graus/segundo\r\n"
        "# Timestamp_ms: Tempo em milissegundos desde o início\r\n"
        "# ===============================\r\n"
        "# Para uso com pandas:\r\n"
        "# import pandas as pd\r\n"
        "# df = pd.read_csv('sensor_data.csv')\r\n"
        "# df['timestamp_s'] = df['Timestamp_ms'] / 1000\r\n"
        "\r\n";
    
    UINT bw;
    res = f_write(&file, metadata, strlen(metadata), &bw);
    f_close(&file);
    
    return (res == FR_OK);
}

bool write_sensor_data(uint32_t sample_num, int16_t accel[3], int16_t gyro[3]) {
    // Durante captura, não mudar LED para evitar flicker
    // Apenas um flash azul muito rápido a cada 50 amostras
    if (sample_num % 50 == 0) {
        led_blue();
        sleep_ms(10);
        led_red();
    }
    
    FIL file;
    FRESULT res = f_open(&file, csv_filename, FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) return false;
    
    // Converter valores brutos para unidades físicas
    // MPU6050 configuração padrão: ±2g para acelerômetro, ±250°/s para giroscópio
    float accel_x_g = accel[0] / 16384.0f;  // LSB/g = 16384 para ±2g
    float accel_y_g = accel[1] / 16384.0f;
    float accel_z_g = accel[2] / 16384.0f;
    
    float gyro_x_dps = gyro[0] / 131.0f;    // LSB/°/s = 131 para ±250°/s
    float gyro_y_dps = gyro[1] / 131.0f;
    float gyro_z_dps = gyro[2] / 131.0f;
    
    // Timestamp em milissegundos desde o início
    uint32_t timestamp = to_ms_since_boot(get_absolute_time());
    
    char buffer[256];
    snprintf(buffer, sizeof(buffer), 
             "%lu,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu\r\n",
             sample_num,                                    // Número da amostra
             accel[0], accel[1], accel[2],                 // Aceleração bruta (LSB)
             gyro[0], gyro[1], gyro[2],                    // Giroscópio bruto (LSB)
             accel_x_g, accel_y_g, accel_z_g,             // Aceleração em g
             gyro_x_dps, gyro_y_dps, gyro_z_dps,          // Giroscópio em °/s
             timestamp);                                    // Timestamp
    
    UINT bw;
    res = f_write(&file, buffer, strlen(buffer), &bw);
    f_close(&file);
    
    return (res == FR_OK);
}

// Interrupções dos botões
void gpio_irq_handler(uint gpio, uint32_t events) {
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Debounce: ignorar interrupções muito próximas (< 200ms)
    if (current_time - last_interrupt_time < 200) return;
    last_interrupt_time = current_time;
    
    if (gpio == BOTAO_A) {
        button_a_pressed = true;
    } else if (gpio == BOTAO_B) {
        button_b_pressed = true;
    }
}

// Função para processar uma única amostra
void process_single_sample() {
    int16_t accel[3], gyro[3];
    mpu6050_read_raw(accel, gyro);
    
    if (write_sensor_data(sample_count + 1, accel, gyro)) {
        sample_count++;
        
        // Atualizar display a cada 10 amostras para não sobrecarregar
        if (sample_count % 10 == 0) {
            ssd1306_fill(&ssd, false);
            
            // Título
            ssd1306_draw_string(&ssd, "STATUS", 0, 0);
            ssd1306_line(&ssd, 0, 10, DISP_W-1, 10, true);
            
            // Status
            ssd1306_draw_string(&ssd, "GRAVANDO", 0, 15);
            ssd1306_draw_string(&ssd, "A:Parar", 0, 25);
            
            // Contador de amostras
            char sample_line[32];
            snprintf(sample_line, sizeof(sample_line), "Amostras: %lu", sample_count);
            ssd1306_draw_string(&ssd, sample_line, 0, 35);
            
            // Nome do arquivo
            char file_line[32];
            snprintf(file_line, sizeof(file_line), "Arq: %s", csv_filename);
            ssd1306_draw_string(&ssd, file_line, 0, 45);
            
            // Status do SD
            //ssd1306_draw_string(&ssd, "SD: OK", 90, 55);
            
            ssd1306_send_data(&ssd);
        }
    } else {
        current_state = STATE_ERROR;
        led_purple();
        update_display("ERRO", "Falha escrita SD");
        capture_active = false;
    }
}

// Inicialização do sistema
void system_init() {
    // Inicializar stdio
    stdio_init_all();
    sleep_ms(2000);
    
    // Gerar nome único para o arquivo
    generate_filename();
    
    // Configurar GPIOs
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    gpio_init(BOTAO_B);
    gpio_set_dir(BOTAO_B, GPIO_IN);
    gpio_pull_up(BOTAO_B);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    // LEDs RGB
    gpio_init(RED_PIN);
    gpio_init(GREEN_PIN);
    gpio_init(BLUE_PIN);
    gpio_set_dir(RED_PIN, GPIO_OUT);
    gpio_set_dir(GREEN_PIN, GPIO_OUT);
    gpio_set_dir(BLUE_PIN, GPIO_OUT);
    
    // Buzzer
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    
    // Estado inicial
    current_state = STATE_INITIALIZING;
    led_yellow();
    
    // Inicializar Display
    i2c_init(I2C_PORT_DISP, 400 * 1000);
    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);
    
    ssd1306_init(&ssd, DISP_W, DISP_H, false, ENDERECO_DISP, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
    
    update_display("Inicializando", "Configurando...");
    
    // Inicializar I2C para MPU6050
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    mpu6050_reset();
    
    update_display("Inicializando", "MPU6050 OK");
    sleep_ms(1000);
    
    // Sistema pronto
    current_state = STATE_READY;
    led_green();
    update_display("PRONTO", "A:Iniciar B:SD");
}

int main() {
    system_init();
    
    uint32_t last_sample_time = 0;
    const uint32_t sample_interval = 50; // 50ms = 20Hz
    
    while (true) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Processar captura de dados se estiver ativo
        if (capture_active && (current_time - last_sample_time >= sample_interval)) {
            process_single_sample();
            last_sample_time = current_time;
        }
        
        // Processar botão A (Iniciar/Parar captura)
        if (button_a_pressed) {
            button_a_pressed = false;
            
            if (!capture_active) {
                // Iniciar captura
                if (!sd_mounted) {
                    current_state = STATE_ERROR;
                    led_purple();
                    update_display("ERRO", "SD nao montado");
                    buzzer_beep(500, 500); // Beep de erro
                    sleep_ms(2000);
                    current_state = STATE_READY;
                    led_green();
                    update_display("PRONTO", "A:Iniciar B:SD");
                } else {
                    sample_count = 0;
                    if (create_csv_file()) {
                        capture_active = true;
                        current_state = STATE_CAPTURING;
                        led_red();
                        buzzer_single_beep();
                        update_display("INICIANDO", "Preparando...");
                        last_sample_time = current_time;
                        sleep_ms(500);
                        update_display("GRAVANDO", "A:Parar");
                    } else {
                        current_state = STATE_ERROR;
                        led_purple();
                        update_display("ERRO", "Falha criar arquivo");
                        buzzer_beep(500, 500);
                        sleep_ms(2000);
                        current_state = STATE_READY;
                        led_green();
                        update_display("PRONTO", "A:Iniciar B:SD");
                    }
                }
            } else {
                // Parar captura
                capture_active = false;
                current_state = STATE_READY;
                led_green();
                buzzer_double_beep();
                char final_info[32];
                snprintf(final_info, sizeof(final_info), "Salvos: %lu", sample_count);
                update_display("FINALIZADO", final_info);
                sleep_ms(3000);
                update_display("PRONTO", "A:Iniciar B:SD");
            }
        }
        
        // Processar botão B (Montar/Desmontar SD)
        if (button_b_pressed) {
            button_b_pressed = false;
            
            if (capture_active) {
                // Não permitir mudanças no SD durante captura
                buzzer_beep(500, 500);
                update_display("ERRO", "Pare captura antes");
                sleep_ms(2000);
                update_display("GRAVANDO", "A:Parar");
                continue;
            }
            
            if (!sd_mounted) {
                // Montar SD
                if (mount_sd_card()) {
                    current_state = STATE_READY;
                    led_green();
                    buzzer_single_beep();
                    update_display("SD MONTADO", "Pronto!");
                    sleep_ms(2000);
                    update_display("PRONTO", "A:Iniciar B:SD");
                } else {
                    current_state = STATE_ERROR;
                    led_purple();
                    buzzer_beep(500, 500);
                    update_display("ERRO SD", "Falha montagem");
                    sleep_ms(2000);
                    current_state = STATE_READY;
                    led_green();
                    update_display("PRONTO", "A:Iniciar B:SD");
                }
            } else {
                // Desmontar SD
                if (unmount_sd_card()) {
                    current_state = STATE_READY;
                    led_green();
                    buzzer_double_beep();
                    update_display("SD DESMONTADO", "Seguro remover");
                    sleep_ms(2000);
                    update_display("PRONTO", "A:Iniciar B:SD");
                } else {
                    current_state = STATE_ERROR;
                    led_purple();
                    buzzer_beep(500, 500);
                    update_display("ERRO", "Falha desmontar");
                    sleep_ms(2000);
                    current_state = STATE_READY;
                    led_green();
                    update_display("PRONTO", "A:Iniciar B:SD");
                }
            }
        }
        
        sleep_ms(10); 
    }
    
    return 0;
}