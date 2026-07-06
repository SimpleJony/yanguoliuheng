#include "image_transfer.h"
#include "search_line.h"

#define IMAGE_TRANSFER_FRAME_PERIOD_MS  (50U)

#if ((MT9V03X_W != 188) || (MT9V03X_H != 120))
#error "Image transfer supports only 188x120 MT9V03X frames"
#endif

/* 图传分辨率减半：188x120 -> 94x60，数据量降为 1/4 */
#define TRANSFER_W                  (MT9V03X_W / 2)     /* 94 */
#define TRANSFER_H                  (MT9V03X_H / 2)     /* 60 */

static uint16 imageTransferElapsedMs = IMAGE_TRANSFER_FRAME_PERIOD_MS;

/* 下采样后的图像缓冲区和边界数组 */
static uint8  transfer_image[TRANSFER_H][TRANSFER_W];
static uint8  transfer_left_edge[TRANSFER_H];
static uint8  transfer_center_line[TRANSFER_H];
static uint8  transfer_right_edge[TRANSFER_H];

/**
 * @brief 将原始 188x120 图像 2x2 块平均下采样为 94x60
 */
static void ImageTransfer_DownSample(void)
{
    uint8 row, col;
    uint16 sum;

    for (row = 0; row < TRANSFER_H; row++)
    {
        for (col = 0; col < TRANSFER_W; col++)
        {
            sum  = mt9v03x_image[row * 2][col * 2];
            sum += mt9v03x_image[row * 2][col * 2 + 1];
            sum += mt9v03x_image[row * 2 + 1][col * 2];
            sum += mt9v03x_image[row * 2 + 1][col * 2 + 1];
            transfer_image[row][col] = (uint8)(sum / 4);
        }
    }
}

/**
 * @brief 将边界数据下采样（每两行取一行，列坐标也减半）
 */
static void ImageTransfer_DownSampleBoundary(void)
{
    uint8 i;

    for (i = 0; i < TRANSFER_H; i++)
    {
        transfer_left_edge[i]   = left_edge_line[i * 2] / 2;
        transfer_center_line[i] = center_line[i * 2] / 2;
        transfer_right_edge[i]  = right_edge_line[i * 2] / 2;
    }
}

void ImageTransfer_Init(void)
{
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                 transfer_image[0],
                                                 TRANSFER_W,
                                                 TRANSFER_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY,
                                              TRANSFER_H,
                                              transfer_left_edge,
                                              transfer_center_line,
                                              transfer_right_edge,
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

    /* 发送前先下采样图像和边界 */
    ImageTransfer_DownSample();
    ImageTransfer_DownSampleBoundary();
    seekfree_assistant_camera_send();

    return 1;
}
