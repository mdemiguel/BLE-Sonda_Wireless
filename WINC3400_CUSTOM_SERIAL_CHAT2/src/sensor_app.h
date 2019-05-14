/*
 * sensor_app.h
 *
 * Created: 08/05/2019 13:51:11
 *  Author: mdemiguel
 */


#ifndef SENSOR_APP_H_
#define SENSOR_APP_H_

 #include <asf.h>
 #include "cscp.h"

 /**
  * prototypes
  */
 void configure_dma_resource(struct dma_resource *resource);
 void setup_transfer_descriptor(DmacDescriptor *descriptor);

 void configure_adc(void);
 void configure_dac(void);
 void configure_dac_channel(void);

 void tc_configuration (void);

 void adc_dma_example(void);

 void sensors_start_job(void);

#define ADC_SAMPLES 150
#define ADC_RESOL_BITS 1024

volatile bool flg_adcRdy;

volatile struct application_flags_t {
		volatile bool adcResult;
}flags;

static struct  adc_vals {

	uint8_t  cycles;
	uint8_t  amplitude;
	uint16_t ain_temperature;
	uint16_t adc_result[ADC_SAMPLES];
	uint16_t adc_us_mv  [ADC_RESOL_BITS];
	uint16_t adc_temp_mv[ADC_RESOL_BITS];
			
}adc_vals;

volatile struct sensor_result{
	volatile char temperature[20];
	volatile char ultrasounds[20];
}sensor_result;


//! [dac_module_inst]
//struct dac_module dac_instance;
//! [dac_module_inst]

//! [adc_module_inst]
struct adc_module adc_instance;
//! [adc_module_inst]

//! [dma_resource]
struct dma_resource example_resource;
//! [dma_resource]

struct tc_module tc_instance;

// [transfer_descriptor]
COMPILER_ALIGNED(16)
DmacDescriptor example_descriptor SECTION_DMAC_DESCRIPTOR;
// [transfer_descriptor]

#endif /* SENSOR_APP_H_ */