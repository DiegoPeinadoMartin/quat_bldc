/*
 * quad_dataSend.h
 *
 *  Created on: Jun 12, 2024
 *      Author: bicho
 */

#ifndef APPLICATIONS_QUAT_QUAT_DATASEND_H_
#define APPLICATIONS_QUAT_QUAT_DATASEND_H_

void quat_send_data_start(void);
void quat_send_data_stop(void);
void quat_send_data_configure(void);
void quat_set_can_msg(uint8_t est1, uint8_t est2, float w1, float w2, float w3, float w2ref1, float w2ref2, float current, float voltage, float factor, bool pendiente);

#endif /* APPLICATIONS_QUAT_QUAT_DATASEND_H_ */
