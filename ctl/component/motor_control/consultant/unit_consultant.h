#ifndef _FILE_UNIT_CONSULTANT_H_
#define _FILE_UNIT_CONSULTANT_H_

/**
 * @brief Calculates permanent magnet flux linkage (Wb) from motor Kv rating.
 * @details 
 * Assumption: Kv is defined as RPM / V_peak_line_to_line.
 * Formula: Flux = 60 / (2 * pi * sqrt(3) * Kv * PolePairs)
 * Constant: 60 / (2 * pi * sqrt(3)) ¡Ö 5.51328895422
 * * @param kv Motor velocity constant (RPM/V)
 * @param pole_pairs Number of pole pairs
 * @return Permanent magnet flux linkage in Webers (Wb)
 */
#define MOTOR_PARAM_CALCULATE_FLUX_BY_KV(kv, pole_pairs) (60.0 / (10.882796185405308 * (kv) * (pole_pairs)))

#endif // _FILE_UNIT_CONSULTANT_H_
