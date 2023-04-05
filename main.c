#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "ADC.h"

void app_main(void)
{
    continuous_adc_init(adc1_chan_mask, adc2_chan_mask, channel, sizeof(channel) / sizeof(adc_channel_t));
    read_sct013_continuous();
    adc_calibration_init();
    single_read();
    Power_Real();
    Power_Reactive();
}
