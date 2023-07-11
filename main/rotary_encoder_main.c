#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <locale.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#define PCNT_HIGH_LIMIT 8193
#define PCNT_LOW_LIMIT -8193
#define GPIO_A GPIO_NUM_4
#define GPIO_B GPIO_NUM_5
#define GPIO_Z GPIO_NUM_6
#define SS1_IN GPIO_NUM_7
#define SS2_IN GPIO_NUM_8
#define PIN_SEL_ZERO ((1ULL << GPIO_Z))
#define PIN_SEL_SENSORES ((1ULL << SS1_IN) | (1ULL << SS2_IN))

static const char *TAG = "Teste_01";
static pcnt_unit_handle_t pcnt_unit = NULL;
static QueueHandle_t queue = NULL;

const double quadratura = 4.0;
const double rot_passo = 1.0;
const double resol_enc = 2048.0 * quadratura;
const double mm_passo = 762.0;

static double d_mm_pulso = 0.0;
static int captura_Entrada_SS1 = 0, captura_Saida_SS1 = 0;
static int captura_Entrada_SS2 = 0, captura_Saida_SS2 = 0;
static bool xcapt_1_Ok = false, xcapt_2_Ok = false;

static void IRAM_ATTR gpio_isr_handler(void *args)
{
	uint32_t pino = (uint32_t)args;
	xQueueSendFromISR(queue, &pino, NULL);
}

void inter_zera_encoder(void *params)
{
	uint32_t pino;
	int cont_max = 0;
	int controle = 0;
	while (1)
	{
		if (xQueueReceive(queue, &pino, portMAX_DELAY))
		{
			if (pino == SS1_IN) {
				if (gpio_get_level(pino)) {
					pcnt_unit_get_count(pcnt_unit, &captura_Entrada_SS1);
					ESP_LOGI("Inter P7", "Captura Entrada SS1_IN = %d", captura_Entrada_SS1);
				}
				else {
					pcnt_unit_get_count(pcnt_unit, &captura_Saida_SS1);
					xcapt_1_Ok = true;
					ESP_LOGI("Inter P7", "Captura Saída SS1_IN = %d", captura_Saida_SS1);
				}
			}
			if (pino == SS2_IN) {
				if (gpio_get_level(pino)) {
					pcnt_unit_get_count(pcnt_unit, &captura_Entrada_SS2);
					ESP_LOGI("Inter P8", "Captura Entrada SS2_IN = %d", captura_Entrada_SS2);
				}
				else {
					pcnt_unit_get_count(pcnt_unit, &captura_Saida_SS2);
					xcapt_2_Ok = true;
					ESP_LOGI("Inter P8", "Captura Saída SS2_IN = %d", captura_Saida_SS2);
				}
			}
			if (pino == GPIO_Z) {
				pcnt_unit_get_count(pcnt_unit, &cont_max);
				pcnt_unit_stop(pcnt_unit);
				pcnt_unit_clear_count(pcnt_unit);
				pcnt_unit_start(pcnt_unit);
				controle++;
				ESP_LOGI("Inter P6", "Salvando e Zerando ==>> Contagem máxima: %d -- Pino: %"PRIu32"  -- "
							"Controle: %d", cont_max, pino, controle);
			}
		} // end master if
	}	  // end while
} // end FB

void f_config_IO(void)
{
	ESP_LOGI(TAG, "install GPIO unit");
	gpio_config_t conf_zero =
		{
			.intr_type = GPIO_INTR_NEGEDGE,
			.mode = GPIO_MODE_INPUT,
			.pin_bit_mask = PIN_SEL_ZERO,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
		};
	gpio_config(&conf_zero);

	gpio_config_t conf_sensores =
		{
			.intr_type = GPIO_INTR_ANYEDGE,
			.mode = GPIO_MODE_INPUT,
			.pin_bit_mask = PIN_SEL_SENSORES,
			.pull_up_en = GPIO_PULLUP_ENABLE,
			.pull_down_en = GPIO_PULLDOWN_DISABLE,
		};
	gpio_config(&conf_sensores);

	queue = xQueueCreate(10, sizeof(uint32_t));
	xTaskCreatePinnedToCore(inter_zera_encoder, "inter_zera_encoder", 8192, NULL, 10, NULL, 0);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(GPIO_Z, gpio_isr_handler, (void *)GPIO_Z);
	gpio_isr_handler_add(SS1_IN, gpio_isr_handler, (void *)SS1_IN);
	gpio_isr_handler_add(SS2_IN, gpio_isr_handler, (void *)SS2_IN);
}

void task_encoder(void *pvParameter)
{
	f_config_IO();
	
	ESP_LOGI(TAG, "install pcnt unit");
	pcnt_unit_config_t unit_config = {
		.high_limit = PCNT_HIGH_LIMIT,
		.low_limit = PCNT_LOW_LIMIT};

	ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

	ESP_LOGI(TAG, "set glitch filter");
	pcnt_glitch_filter_config_t filter_config = {
		.max_glitch_ns = 12000};
	ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

	ESP_LOGI(TAG, "install pcnt channels");
	pcnt_chan_config_t chan_a_config = {
		.edge_gpio_num = GPIO_A,
		.level_gpio_num = GPIO_B};
	pcnt_channel_handle_t pcnt_chan_a = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

	pcnt_chan_config_t chan_b_config = {
		.edge_gpio_num = GPIO_B,
		.level_gpio_num = GPIO_A};
	pcnt_channel_handle_t pcnt_chan_b = NULL;
	ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

	ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

	ESP_LOGI(TAG, "enable pcnt unit");
	ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
	ESP_LOGI(TAG, "clear pcnt unit");
	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
	ESP_LOGI(TAG, "start pcnt unit");
	ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

	//int pulse_count = 0;
	//int pulse_count_old = 0;

	while (1)
	{
		/*
				ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
				if (pulse_count_old != pulse_count)
				{
					ESP_LOGI(TAG, "Pulse count: %d", pulse_count);
					pulse_count_old = pulse_count;
				}	// END IF
		*/
		vTaskDelay(10 / portTICK_PERIOD_MS);

	} //	END WHILE
} // END Routine

void efetuaCalculo(void *pvParameters)
{
	double cota_SS1 = 0.0;
	double cota_SS2 = 0.0;
	double variacao1 = 0.0;
	double variacaoEntr = 0.0;
	double variacaoSaida = 0.0;
	while(1)
	{
		if (xcapt_1_Ok & xcapt_2_Ok)
		{
			cota_SS1 = 1.0 * ((captura_Saida_SS1 - captura_Entrada_SS1) * d_mm_pulso);
			cota_SS2 = 1.0 * ((captura_Saida_SS2 - captura_Entrada_SS2) * d_mm_pulso);
			variacao1 = abs(cota_SS1 - cota_SS2);
			variacaoEntr = abs(captura_Entrada_SS1 - captura_Entrada_SS2) * d_mm_pulso;
			variacaoSaida = abs(captura_Saida_SS1 - captura_Saida_SS2) * d_mm_pulso;
			ESP_LOGI("Cálculo SS1", "Cota SS1: %2.3f", cota_SS1);
			ESP_LOGI("Cálculo SS2", "Cota SS2: %2.3f", cota_SS2);
			ESP_LOGI("Variação SS1/SS2", "Variação Cota: %2.3f", variacao1);
			ESP_LOGI("Variação Entrada", "Variação da Entrada: %2.3f", variacaoEntr);
			ESP_LOGI("Variação Saída", "Variação da Saída: %2.3f", variacaoSaida);
			xcapt_1_Ok = false;
			xcapt_2_Ok = false;
		}
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

void app_main(void)
{
	d_mm_pulso = mm_passo / (resol_enc * rot_passo);
    printf("Encoder: milímetro por pulso = %2.7F \n", d_mm_pulso);
	xTaskCreatePinnedToCore(&task_encoder, "task_encoder", 8192, NULL, 1, NULL, 1);
	xTaskCreatePinnedToCore(&efetuaCalculo, "task_calculo", 8192, NULL, 1, NULL, 0);
	//vTaskStartScheduler();
}
