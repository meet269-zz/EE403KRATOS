/*
 * ring_buffer.h
 *
 *  Created on: Mar 23, 2017
 *      Author: mattp
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

typedef struct
{
	uint32_t index;
	float input[3];
	float output[3];
	float num[3];
	float den[3];
} ring_buffer3;

void rb_init(ring_buffer3 *rb);
void rb_push(ring_buffer3 *rb, float SampleIn);

#endif /* RING_BUFFER_H_ */
