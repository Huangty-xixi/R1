#ifndef __COWORK_H__
#define __COWORK_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "policy.h"
#include "remote_control.h"
#include "main.h"

extern bool sent_flag;
extern uint8_t uart8_tx_buf[8];
extern uint8_t four_sent[4];
extern uint8_t th_sent[5];

extern bool led01;
extern bool led02;

extern void uart8_tx_init(void);
extern void fir_dismiss(void);
extern void th_order(void);
extern void th_5mode(void);

extern void sense(void);
extern void SwapNumArray(int *buf, int len);
static uint8_t combine_4bit(uint8_t high_4bit, uint8_t low_4bit);
extern void JudgeRelativeSize_Global(int kfs1[3], float removed_k1);

#endif
