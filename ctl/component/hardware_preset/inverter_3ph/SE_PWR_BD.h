
#ifndef _FILE_SE_PWR_BD_H_
#define _FILE_SE_PWR_BD_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Current output when ADC input is ADC_REFERENCE, unit A
//#define ADC_FULLSCALE_CURRENT ((42.66/2))
#define ADC_FULLSCALE_CURRENT ((40))

// Voltage output when ADC input is ADC_REFERENCE, unit V
#define ADC_FULLSCALE_VOLTAGE ((617.18))

// Current ADC reference voltage, unit V
#define ADC_CURRENT_REFERENCE_VOLTAGE ((0.16))

// Voltage ADC reference voltage, unit V
#define ADC_VOLTAGE_REFERENCE_VOLTAGE ((0))

// Given Full scale voltage reference, unit V
#define ADC_REFERENCE ((0.32))

#ifdef __cplusplus
}
#endif // __cplusplus

/*
Current Calibration									
PWR_BD				100W			400W			750W			1000W			1500W			2kW				3kW				5kW				test
Rs(ohm)				0.025			0.015			0.0075			0.005			0.004			0.0025			0.002			0.0015			0.006
SD_FULLDATA			320
m_iqIuMeasGain		0.042666667		0.071111111		0.142222222		0.213333333		0.266666667		0.426666667		0.533333333		0.711111111		0.177777778
uiIu_usr_sc			1280			2133			4266			6400			8000			12800			16000			21333			5333
uiIu_usr_sc_min		1216			2026.35			4052.7			6080			7600			12160			15200			20266.35		5066.35
uiIu_usr_sc_max		1344			2239.65			4479.3			6720			8400			13440			16800			22399.65		5599.65
*/



#endif // _FILE_SE_PWR_BD_H_
