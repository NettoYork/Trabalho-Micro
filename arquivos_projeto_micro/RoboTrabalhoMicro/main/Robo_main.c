#include "driver/gpio.h"
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdlib.h>
#include "driver/uart.h" 
#include <math.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include <time.h>

#define SERVO_MIN_PULSEWIDTH 500  
#define SERVO_MAX_PULSEWIDTH 2500 
#define SERVO_MAX_DEGREE 180      

#define STEP_SIZE 18              
#define ARRAY_SIZE (SERVO_MAX_DEGREE / STEP_SIZE) 

#define UART_NUM UART_NUM_1
#define TX_PIN GPIO_NUM_17
#define RX_PIN GPIO_NUM_16

#define ENCODER_PIN_A GPIO_NUM_19
#define ENCODER_PIN_B GPIO_NUM_18
#define ENCODER_PIN_C GPIO_NUM_22
#define ENCODER_PIN_D GPIO_NUM_23
#define LED_PIN GPIO_NUM_2
#define front_PIN GPIO_NUM_27
#define back_PIN GPIO_NUM_26
#define front_PIN1 GPIO_NUM_13
#define back_PIN1 GPIO_NUM_12
#define TRIGGER_PIN GPIO_NUM_32
#define ECHO_PIN GPIO_NUM_33

#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_CHANNEL1   LEDC_CHANNEL_1
#define LEDC_CHANNEL2   LEDC_CHANNEL_2
#define LEDC_CHANNEL3   LEDC_CHANNEL_3
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT // Resolução do PWM de 10 bits
#define LEDC_FREQUENCY  5000
#define MAX_DISTANCE_CM 400

typedef struct {
    int angle;
    float reading;
} dir;

dir outward_dir[ARRAY_SIZE];
dir steering_back[ARRAY_SIZE];

void mcpwm_example_gpio_initialize() {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 4);  
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + ((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE));
    return cal_pulsewidth;
}

static int16_t encoder_count = 0;
static int16_t encoder_count1 = 0;
static const float COLLISION_DISTANCE_THRESHOLD = 20.0; // Distância em centímetros


void init_uart() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_NUM, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

static void IRAM_ATTR encoder_isr_handler(void *arg) {
    static int8_t last_state = 0;
    int8_t current_state = (gpio_get_level(ENCODER_PIN_A) << 1) | gpio_get_level(ENCODER_PIN_B);

    if (current_state != last_state) {
        if ((last_state == 0 && current_state == 2) ||
            (last_state == 2 && current_state == 3) ||
            (last_state == 3 && current_state == 1) ||
            (last_state == 1 && current_state == 0)) {
            encoder_count++;
        } else if ((last_state == 0 && current_state == 1) ||
                   (last_state == 1 && current_state == 3) ||
                   (last_state == 3 && current_state == 2) ||
                   (last_state == 2 && current_state == 0)) {
            encoder_count--;
        }
        last_state = current_state;
    }
}

static void IRAM_ATTR encoder_isr_handler1(void *arg) {
    static int8_t last_state = 0;
    int8_t current_state = (gpio_get_level(ENCODER_PIN_C) << 1) | gpio_get_level(ENCODER_PIN_D);

    if (current_state != last_state) {
        if ((last_state == 0 && current_state == 2) ||
            (last_state == 2 && current_state == 3) ||
            (last_state == 3 && current_state == 1) ||
            (last_state == 1 && current_state == 0)) {
            encoder_count1++;
        } else if ((last_state == 0 && current_state == 1) ||
                   (last_state == 1 && current_state == 3) ||
                   (last_state == 3 && current_state == 2) ||
                   (last_state == 2 && current_state == 0)) {
            encoder_count1--;
        }
        last_state = current_state;
    }
}

void init_project() {

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENCODER_PIN_A) | (1ULL << ENCODER_PIN_B) | (1ULL << ENCODER_PIN_C) | (1ULL << ENCODER_PIN_D),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_PIN_A, encoder_isr_handler, NULL);
    gpio_isr_handler_add(ENCODER_PIN_B, encoder_isr_handler, NULL);
    gpio_isr_handler_add(ENCODER_PIN_C, encoder_isr_handler1, NULL);
    gpio_isr_handler_add(ENCODER_PIN_D, encoder_isr_handler1, NULL);

    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(front_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(back_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(front_PIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(back_PIN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_FADE_END,
        .gpio_num = front_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL1;
    ledc_channel.gpio_num = back_PIN;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL2;
    ledc_channel.gpio_num = front_PIN1;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel = LEDC_CHANNEL3;
    ledc_channel.gpio_num = back_PIN1;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

}

float measure_distance() {
    // Gera o pulso de trigger
    gpio_set_level(TRIGGER_PIN, 0);
    usleep(2);
    gpio_set_level(TRIGGER_PIN, 1);
    usleep(10);
    gpio_set_level(TRIGGER_PIN, 0);

    // Variáveis para armazenar o tempo
    int64_t start_time = 0;
    int64_t end_time = 0;

    // Espera o início do pulso do Echo
    int64_t wait_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0 && (esp_timer_get_time() - wait_time) < 1000000) {
        start_time = esp_timer_get_time();
    }

    // Espera o fim do pulso do Echo
    wait_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1 && (esp_timer_get_time() - wait_time) < 1000000) {
        end_time = esp_timer_get_time();
    }

    // Calcula a duração do pulso
    int64_t duration = end_time - start_time;

    // Calcula a distância em centímetros
    float distance = duration / 58.0;

    // Retorna a distância calculada
    return distance;
}

void move(int v1, int v2, int condition){

    int duty_cycle_fren = 0;
    int duty_cycle_back = 0;
    int duty_cycle_fren1 = 0;
    int duty_cycle_back1 = 0;
    if(condition > 0){
      if(v1 > 0){
        duty_cycle_fren = v1;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle_fren));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL1, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL1));
    } else {
        duty_cycle_back = -v1;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL1, duty_cycle_back));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL1));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    }

    if(v2 > 0){
        duty_cycle_fren1 = v2;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL2, duty_cycle_fren1));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL2));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL3, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL3));
    } else {
        duty_cycle_back1 = -v2;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL3, duty_cycle_back1));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL3));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL2, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL2));
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}
        
    // Parar os motores após o tempo especificado
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL1));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL2, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL3, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL3));
}

void move_right(int condition){
    move(-200, 200, condition);
}

void move_left(int condition){
    move(200, 200, condition);
}
void back(int condition){
    move(-180, -180, condition);
}
void front(int v1, int v2, int condition){
    move(v1, v2, condition);
}

dir scan_distance(){
    dir max;
    mcpwm_example_gpio_initialize();
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    
    pwm_config.cmpr_a = 0;        
    pwm_config.cmpr_b = 0;
        
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    

    int max_reading = -1;
    int max_angle = 0;
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(90));
    int index = 0;
    for (int j = 0; j < SERVO_MAX_DEGREE; j += STEP_SIZE) {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(j));
        outward_dir[index].angle = j;
        outward_dir[index].reading = measure_distance();
        
        if (outward_dir[index].reading > max_reading) {
            max_reading = outward_dir[index].reading;
            max_angle = j;
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        index++;
    }

    index = 0;

    for (int t = SERVO_MAX_DEGREE; t > 0; t -= STEP_SIZE) {
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(t));
        steering_back[index].angle = t;
        steering_back[index].reading = measure_distance();
        
        if (steering_back[index].reading > max_reading) {
            if(t > max_angle){
                max_reading = steering_back[index].reading;
                max_angle = t;
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
        index++;
    }
    // Move o servo para a posição correspondente ao maior valor de reading
    //mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(max_angle));
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, servo_per_degree_init(90));
    max.angle = max_angle;
    max.reading = max_reading;
    return max;
}

void motor_control_task(void *pvParameter) {

    while (1) {
        float distance = measure_distance();

        if(distance > COLLISION_DISTANCE_THRESHOLD) {
            // Atualizar velocidades conforme valores do vetor
            int vel1 = 200;
            int vel2 = 190;
            front(vel1, vel2, 1);
        }
        else if (distance < COLLISION_DISTANCE_THRESHOLD) {
            // Manobra para virar à direita
            dir giro = scan_distance();
            if(giro.angle > 90){
                if(giro.reading - distance > 8.0){
                    move_right(1);
                }
                else{
                    move_right(0);
                }
            }
            else if(giro.angle < 90){
                if(giro.reading - distance > 8.0){
                    move_left(1); 
                }
                else{
                    move_left(0); 
                }
            }
            else{
                if(measure_distance() < 100){
                    back(1);
                }
                back(0);
            }
            // Esperar um pouco para completar a manobra
        } 

        // Enviar os dados via UART
        char message[256];
        int message_len = snprintf(message, sizeof(message), " %.2f             %d               %d\n", 
                                    distance, encoder_count, encoder_count1);
        uart_write_bytes(UART_NUM, message, message_len);

        // Imprimir no terminal para verificação
        printf("%s", message);
        vTaskDelay(pdMS_TO_TICKS(200));

    }
}


void app_main(void) {
    
    init_project();
    init_uart();
    xTaskCreate(&motor_control_task, "motor_control_task", 4096, NULL, 5, NULL);
}