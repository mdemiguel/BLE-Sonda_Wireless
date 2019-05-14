/*
 * sensor_app.c
 *
 * Created: 08/05/2019 13:51:57
 *  Author: mdemiguel
 */
#include "sensor_app.h"
#include "stdio.h"

static inline void tc_adc_trigger_cb (struct tc_module *const module_inst)
{
	/* 
	 * Activando los bits del ADC y del puerto directamente en los
	 * registros, se evita el "overhead" de utilizar muchas llamadas
	 */	
	adc_vals.cycles++;
	if (adc_vals.cycles == ADC_SAMPLES) {

		uint32_t us_media   = 0;
		uint32_t temp_media = 0;
		uint32_t basura     = 0;
		uint8_t  i          = 0;
		
		/* Para conversión controlada a través de Android, 
		 * desactivar el Timer.
		 */
		//tc_disable(&tc_instance);

		while (i<ADC_SAMPLES){
			i+=3;
			basura     += adc_vals.adc_result[i-3]; // Aquí se almacena un canal vacío en el caso del LT4.
			temp_media += adc_vals.adc_result[i-2];
			us_media   += adc_vals.adc_result[i-1];
			asm("nop");
		}
		us_media   /= 50;							// Valor medio de los ultrasonidos medidos
		temp_media /= 50;
		/*basura /= 50;*/							// Valor medio de temperatura

		adc_vals.cycles = 0;						// Restart counter value
		us_media   *= 1.61; 
		temp_media *= 1.61;		
		TC3->COUNT8.COUNT.reg = 0x00; 
				
		sprintf(sensor_result.temperature, "Temperature: %lu mV\r\n", temp_media);
		sprintf(sensor_result.ultrasounds, "Ultrasounds: %lu mV\r\n", us_media);
		
		flg_adcRdy = true;
		asm("nop");
	}
	
	// Lanza una conversión cada 24 uS
	ADC->SWTRIG.reg |= ADC_SWTRIG_START;   // Quita 4 o 5us de retardo hacerlo así

	PORT->Group[1].OUTTGL.reg = PORT_PB30; // Sólo está para comprobar tiempos*/
}

static void dma_tx_done_cb(struct dma_resource *const resource)
{
	asm("nop");
}

void tc_configuration (void)
{
	/*	8 MHz / 16 = 500 kHz, T=2uS */
	/* 48 MHz / 64 = 750 kHz  T= 1.33uS*/
	
	struct tc_config config_tc;

	/* 1. Get config defaults */
	tc_get_config_defaults(&config_tc);

	/* 2. Timer Config  */
	config_tc.counter_size         = TC_COUNTER_SIZE_8BIT;
	config_tc.clock_source         = GCLK_GENERATOR_3;
	config_tc.clock_prescaler      = /*TC_CLOCK_PRESCALER_DIV64*/ TC_CLOCK_PRESCALER_DIV16;
	config_tc.counter_8_bit.period = 12; // 2uS*12 = 24uS+delay	if 8Mhz/16
										 // 1.3333us*18 if  48MHz/64
	//config_tc.counter_8_bit.compare_capture_channel[0] = 12;

	/* 3. Init. hardware timer TC3 */
	tc_init(&tc_instance, TC3, &config_tc);

	tc_enable(&tc_instance);

	tc_register_callback(&tc_instance, tc_adc_trigger_cb, TC_CALLBACK_OVERFLOW);

	tc_enable_callback  (&tc_instance, TC_CALLBACK_OVERFLOW);

}


/** 
 * AIN[16] = PA08, AIN[17] = PA09 , AIN[18] = PA10
 *
 */
void configure_adc(void)
{
	struct adc_config config_adc;

	/* Para ahorrar tiempo de cálculo al micro
	 * se almacenan en un array todos los posibles valores
	 * en mV desde 0 a 2^12-1 bits (resolución adc).
	 * Las constantes se calculan a partir de la función de
	 * conversión del ADC, que depende de la resolución y la
	 * tensión de referencia.
	 */
	adc_vals.cycles          = 0;
	adc_vals.amplitude        = 0;
	adc_vals.ain_temperature = 0;
	flg_adcRdy = false;

	//for (uint16_t i =0; i<ADC_RESOL_BITS; i++){
		///* Este valor es de amplitud. Está calculado para que
		 //* cuando se le pide una posición i que se correponde
		 //* con uno de los 4095 valores medios posibles, se
		 //* muestre el valor en mV de amplitud de una señal
		 //* senoidal.
		 //*/
		//adc_vals.adc_us_mv[i] = i*0.447398312 + 0.108961704;
	//}
	//asm("nop");

	adc_get_config_defaults(&config_adc);

	config_adc.clock_source    = GCLK_GENERATOR_3;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV4;
	config_adc.resolution      = ADC_RESOLUTION_10BIT; 
	config_adc.reference       = ADC_REFERENCE_INTVCC1;
	config_adc.gain_factor     = ADC_GAIN_FACTOR_2X;
	config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN16;
	//config_adc.negative_input  = ADC_NEGATIVE_INPUT_PIN5;
	//config_adc.differential_mode = true;
	config_adc.freerunning     = false;
	config_adc.left_adjust     = false;
	config_adc.pin_scan.offset_start_scan    = 0;
	config_adc.pin_scan.inputs_to_scan       = 3;

	adc_init(&adc_instance, ADC, &config_adc);

	adc_enable(&adc_instance);

    asm("nop");

}// FIN

void configure_dma_resource(struct dma_resource *resource)
{

	struct dma_resource_config config;

	dma_get_config_defaults(&config);

	config.peripheral_trigger = ADC_DMAC_ID_RESRDY;
	config.trigger_action     = DMA_TRIGGER_ACTION_BEAT;

	dma_register_callback(resource, dma_tx_done_cb, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback  (resource, DMA_CALLBACK_TRANSFER_DONE);

	dma_allocate(resource, &config);

	NVIC_DisableIRQ(DMAC_IRQn);
	NVIC_ClearPendingIRQ(DMAC_IRQn);
	NVIC_EnableIRQ(DMAC_IRQn);
}

void setup_transfer_descriptor(DmacDescriptor *descriptor)
{
	struct dma_descriptor_config descriptor_config;

	dma_descriptor_get_config_defaults(&descriptor_config);

	descriptor_config.beat_size = DMA_BEAT_SIZE_HWORD;
	descriptor_config.dst_increment_enable    = true;
	descriptor_config.src_increment_enable    = false;
	descriptor_config.block_transfer_count    = ADC_SAMPLES;
	descriptor_config.source_address		  = (uint32_t)(&adc_instance.hw->RESULT.reg);
	descriptor_config.destination_address     = (uint32_t)(&adc_vals.adc_result[0]+ADC_SAMPLES);//(uint32_t)(&dac_instance.hw->DATA.reg);
	descriptor_config.next_descriptor_address = (uint32_t)descriptor;

	dma_descriptor_create(descriptor, &descriptor_config);
}

void adc_dma_example(void)
{
	configure_adc();

 	configure_dma_resource(&example_resource);

 	setup_transfer_descriptor(&example_descriptor);

 	dma_add_descriptor(&example_resource, &example_descriptor);

	adc_start_conversion(&adc_instance);
	
	tc_configuration();
	
	dma_start_transfer_job(&example_resource);
}

void sensors_start_job(void)
{
	asm("nop");
	//adc_start_conversion(&adc_instance);
	tc_configuration();
	dma_start_transfer_job(&example_resource);
}

void sensors_stop_job (void)
{
	tc_disable(&tc_instance);
}