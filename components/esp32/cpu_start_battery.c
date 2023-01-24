#include "esp_log.h"


#include "esp_vfs_fat.h"
#include "driver/adc.h"
#include "soc/rtc_io_reg.h"
#include "soc/rtc_io_struct.h"
#include "soc/sens_reg.h"
#include "soc/sens_struct.h"
#include "cpu_start_battery.h"

static const char *TAG = "singleCore12VBattery";

RTC_NOINIT_ATTR uint32_t battery12vinfopacked1 = 0xfefefefe;
RTC_NOINIT_ATTR uint32_t battery12vinfopacked2 = 0xefefefef;
RTC_NOINIT_ATTR uint8_t battery12vWakeVoltages[BATTERY12V_WAKE_VOLTAGE_COUNT];
RTC_NOINIT_ATTR uint32_t battery12vWakeCounter = 0;


float singleCoreCurrentBatteryLevelAvg()
{
    ESP_EARLY_LOGI(TAG,"singleCoreReadBatteryLevel" );
#ifdef CONFIG_OVMS_COMP_ADC
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_11db);

         RTCIO.hall_sens.xpd_hall = false;
    SENS.sar_meas_wait2.force_xpd_amp = SENS_FORCE_XPD_AMP_PD;
    //disable FSM, it's only used by the LNA.
    SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;
    SENS.sar_meas_wait1.sar_amp_wait1 = 1;
    SENS.sar_meas_wait1.sar_amp_wait2 = 1;
    SENS.sar_meas_wait2.sar_amp_wait3 = 1;
    //set controller
    SENS.sar_read_ctrl.sar1_dig_force = false;      //RTC controller controls the ADC, not digital controller
    SENS.sar_meas_start1.meas1_start_force = true;  //RTC controller controls the ADC,not ulp coprocessor
    SENS.sar_meas_start1.sar1_en_pad_force = true;  //RTC controller controls the data port, not ulp coprocessor
    SENS.sar_touch_ctrl1.xpd_hall_force = true;     // RTC controller controls the hall sensor power,not ulp coprocessor
    SENS.sar_touch_ctrl1.hall_phase_force = true;   // RTC controller controls the hall sensor phase,not ulp coprocessor
    //start conversion

    uint32_t min = 4294967295;
    uint32_t max = 0;
    uint32_t sum = 0;

    for (int i = 0; i < 7; i++)
    {
        uint16_t adc_value;

        SENS.sar_meas_start1.sar1_en_pad = (1 << ADC1_CHANNEL_0); //only one channel is selected.
        while (SENS.sar_slave_addr1.meas_status != 0);
        SENS.sar_meas_start1.meas1_start_sar = 0;
        SENS.sar_meas_start1.meas1_start_sar = 1;
        while (SENS.sar_meas_start1.meas1_done_sar == 0);
        adc_value = SENS.sar_meas_start1.meas1_data_sar;

        uint32_t value = adc_value;

        min = value < min ? value : min;
        max = value > max ? value : max;

        sum += value;
    }
    sum -= min;
    sum -= max;

    float avg = ((float)sum) / 5.0f;
    ESP_EARLY_LOGI(TAG, "min:%d max:%d sum:%d %d",min,max,sum,(int)(avg * 10));
    return avg;
#else
  ESP_EARLY_LOGI(TAG, "ADC not available, cannot check 12V level");
  return 2544.0;
#endif // CONFIG_OVMS_COMP_ADC
}


float singleCoreCurrentBatteryVoltageAdjusted(float calibratedVoltage, float calibratedLevel)
{
    float   low11v  = 1967.0; float low15v  = 2787.5;
    float   high11v = 2200.5; float high15v = 3098.0;

    float   lowascending  = ( low15v  - low11v  ) / ( 15.0 - 11.0 );
    float   highascending = ( high15v - high11v ) / ( 15.0 - 11.0 );

    float   lowlevel  = low11v  + ( (calibratedVoltage - 11.0) * lowascending );
    float   highlevel = high11v + ( (calibratedVoltage - 11.0) * highascending );

    float   ratiolowhigh = ( calibratedLevel - lowlevel) /  ( highlevel - lowlevel );

    float   ascending = lowascending + ( (highascending - lowascending) * ratiolowhigh ) ;

    float   currentlevel = (float)singleCoreCurrentBatteryLevelAvg();

    float currentVoltage = calibratedVoltage - ( (calibratedLevel - currentlevel) / ascending ) ;

    return currentVoltage;
}

bool singleCoreVoltageIsInAcceptableRange(float currentVoltage,float alertVoltage)
{
    if( currentVoltage > alertVoltage )
    {
        return true;
    }

    if( currentVoltage < 5.0 )
    {
        ESP_EARLY_LOGI(TAG, "Assuming USB powered, proceeding with boot");
        return true;
    }
    return false;
}

void addVoltageToMemory(const uint8_t currentVoltage);
void singleCoreSleepBatteryIfNeeded()
{
    const uint32_t packedvalue1 = battery12vinfopacked1;
    const uint32_t packedvalue2 = battery12vinfopacked2;

    if(  0 == packedvalue1  || packedvalue1 != packedvalue2)
    {
        ESP_EARLY_LOGI(TAG,"invalid packedvalue1 != packedvalue2 %d!=%d",packedvalue1,packedvalue2);
        //battery12vWakeCounter=0;
        battery12vWakeVoltages[battery12vWakeCounter++ % BATTERY12V_WAKE_VOLTAGE_COUNT] = 255;
        battery12vWakeVoltages[battery12vWakeCounter++ % BATTERY12V_WAKE_VOLTAGE_COUNT] = 251;
        battery12vWakeVoltages[battery12vWakeCounter++ % BATTERY12V_WAKE_VOLTAGE_COUNT] = 255;
        battery12vWakeVoltages[battery12vWakeCounter++ % BATTERY12V_WAKE_VOLTAGE_COUNT] = 252;
        battery12vWakeVoltages[battery12vWakeCounter++ % BATTERY12V_WAKE_VOLTAGE_COUNT] = 255;
        battery12vWakeVoltages[battery12vWakeCounter++ % BATTERY12V_WAKE_VOLTAGE_COUNT] = 253;
        battery12vWakeVoltages[battery12vWakeCounter++ % BATTERY12V_WAKE_VOLTAGE_COUNT] = 255;
        battery12vWakeVoltages[battery12vWakeCounter++ % BATTERY12V_WAKE_VOLTAGE_COUNT] = BATTERY12V_SLEEP_TIME;
        battery12vWakeVoltages[battery12vWakeCounter % BATTERY12V_WAKE_VOLTAGE_COUNT] = 0;
        battery12vWakeVoltages[(battery12vWakeCounter+1) % BATTERY12V_WAKE_VOLTAGE_COUNT] = 0;
        return;
    }
    ESP_EARLY_LOGI(TAG,"packedvalue: %d %d",(packedvalue1 >> 16),(packedvalue1 & 0xFFFF));
    const float alertVoltage        = ((float)(packedvalue1 >> 16)) / 1000.0;
    const float calibrationFactor   = ((float)(packedvalue1 & 0xFFFF)) / 10.0;


    const float currentVoltage = singleCoreCurrentBatteryVoltageAdjusted(alertVoltage,alertVoltage * calibrationFactor);
    ESP_EARLY_LOGI(TAG,"currentVoltage: %d",(int)(currentVoltage * 10.0));

    addVoltageToMemory( (int)(currentVoltage * 10.0) );

    if( !singleCoreVoltageIsInAcceptableRange(currentVoltage,alertVoltage))
    {
        ESP_EARLY_LOGI(TAG,"not acceptable");

        battery12vinfopacked1 = packedvalue1;
        battery12vinfopacked1 = packedvalue1;

        esp_deep_sleep(1500000LL * BATTERY12V_SLEEP_TIME);
    }
    battery12vWakeCounter += 2;
    battery12vWakeVoltages[battery12vWakeCounter % BATTERY12V_WAKE_VOLTAGE_COUNT] = 255;
    battery12vWakeVoltages[(battery12vWakeCounter+1) % BATTERY12V_WAKE_VOLTAGE_COUNT] = 255;

    ESP_EARLY_LOGI(TAG,"is acceptable");
}


void addVoltageToMemory(const uint8_t currentVoltage)
{
    uint32_t voltageposition  = battery12vWakeCounter % BATTERY12V_WAKE_VOLTAGE_COUNT;
    uint32_t counterposition  = (battery12vWakeCounter+1) % BATTERY12V_WAKE_VOLTAGE_COUNT;

    if(     battery12vWakeVoltages[voltageposition] == currentVoltage
        &&  battery12vWakeVoltages[counterposition] < 254
      )
    {
        battery12vWakeVoltages[counterposition] = battery12vWakeVoltages[counterposition] + 1;
    }
    else
    {
        battery12vWakeCounter +=2;
        voltageposition  = battery12vWakeCounter % BATTERY12V_WAKE_VOLTAGE_COUNT;
        counterposition  = (battery12vWakeCounter+1) % BATTERY12V_WAKE_VOLTAGE_COUNT;

        battery12vWakeVoltages[voltageposition] = currentVoltage;
        battery12vWakeVoltages[counterposition] = 1;
    }
    ESP_EARLY_LOGI(TAG,"position:%d voltage:%d count:%d currentVoltage:%d ",counterposition,battery12vWakeVoltages[voltageposition],battery12vWakeVoltages[counterposition],currentVoltage);
}
