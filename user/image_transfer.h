#ifndef IMAGE_TRANSFER_H
#define IMAGE_TRANSFER_H

#include "init.h"

void ImageTransfer_Init(void);
void ImageTransfer_Tick(uint16 elapsedMs);
uint8 ImageTransfer_SendFrame(void);

#endif
