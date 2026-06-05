#include "image_transfer.h"
#include "search_line.h"

#define IMAGE_TRANSFER_FRAME_PERIOD_MS  (50U)

#if ((MT9V03X_W != 188) || (MT9V03X_H != 120))
#error "Image transfer supports only 188x120 MT9V03X frames"
#endif

static uint16 imageTransferElapsedMs = IMAGE_TRANSFER_FRAME_PERIOD_MS;

void ImageTransfer_Init(void)
{
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                 mt9v03x_image[0],
                                                 MT9V03X_W,
                                                 MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY,
                                              SEARCH_IMAGE_H,
                                              left_edge_line,
                                              center_line,
                                              right_edge_line,
                                              NULL,
                                              NULL,
                                              NULL);
}

void ImageTransfer_Tick(uint16 elapsedMs)
{
    if (imageTransferElapsedMs < (60000U - elapsedMs))
    {
        imageTransferElapsedMs += elapsedMs;
    }
}

uint8 ImageTransfer_SendFrame(void)
{
    if (imageTransferElapsedMs < IMAGE_TRANSFER_FRAME_PERIOD_MS)
    {
        return 0;
    }

    imageTransferElapsedMs = 0;
    seekfree_assistant_camera_send();

    return 1;
}
