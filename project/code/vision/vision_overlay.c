#include "vision/vision_overlay.h"
#include "vision/ipm_interface.h"
#include "vision/vision_runtime.h"

static uint8 VisionOverlay_GetBinaryPixel(const uint8 *binary_image,
                                          int col, int row)
{
    uint8 mask = (uint8)(0x80U >> (col & 7));
    return (uint8)((binary_image[row * VISION_RUNTIME_BINARY_STRIDE +
                                (col >> 3)] & mask) != 0U);
}

static uint8 VisionOverlay_IsImagePointInBounds(int img_x, int img_y)
{
    if(img_x < 0 || img_x >= MT9V03X_W)
    {
        return 0U;
    }

    if(img_y < 0 || img_y >= MT9V03X_H)
    {
        return 0U;
    }

    return 1U;
}

uint8 VisionOverlay_WorldToImage(float world_x, float world_y, int *img_x, int *img_y)
{
    int proj_x = 0;
    int proj_y = 0;

    if(img_x == NULL || img_y == NULL)
    {
        return 0U;
    }

    if(!IPM_World2RectifiedImage(world_x, world_y, &proj_x, &proj_y))
    {
        return 0U;
    }

    if(!VisionOverlay_IsImagePointInBounds(proj_x, proj_y))
    {
        return 0U;
    }

    *img_x = proj_x;
    *img_y = proj_y;
    return 1U;
}

uint8 VisionOverlay_WorldLineToImage(float world_x0, float world_y0,
                                     float world_x1, float world_y1,
                                     int *img_x0, int *img_y0,
                                     int *img_x1, int *img_y1)
{
    if(img_x0 == NULL || img_y0 == NULL || img_x1 == NULL || img_y1 == NULL)
    {
        return 0U;
    }

    if(!VisionOverlay_WorldToImage(world_x0, world_y0, img_x0, img_y0))
    {
        return 0U;
    }

    if(!VisionOverlay_WorldToImage(world_x1, world_y1, img_x1, img_y1))
    {
        return 0U;
    }

    return 1U;
}

void VisionOverlay_DrawImagePoint4x4(int img_x, int img_y, uint16 color, uint16 image_y_offset)
{
    int dx = 0;
    int dy = 0;

    for(dy = -1; dy <= 2; dy++)
    {
        for(dx = -1; dx <= 2; dx++)
        {
            int px = img_x + dx;
            int py = img_y + dy;

            if(!VisionOverlay_IsImagePointInBounds(px, py))
            {
                continue;
            }

            ips200_draw_point((uint16)px, (uint16)(py + (int)image_y_offset), color);
        }
    }
}

void VisionOverlay_DrawFineImagePoint4x4(int img_x, int img_y,
                                         uint16 color, uint16 image_y_offset)
{
    VisionOverlay_DrawImagePoint4x4(img_x / 2, img_y / 2,
                                    color, image_y_offset);
}

void VisionOverlay_DrawFineImageLine(int img_x0, int img_y0,
                                     int img_x1, int img_y1,
                                     uint16 color, uint16 image_y_offset)
{
    ips200_draw_line((uint16)(img_x0 / 2),
                     (uint16)(img_y0 / 2 + (int)image_y_offset),
                     (uint16)(img_x1 / 2),
                     (uint16)(img_y1 / 2 + (int)image_y_offset),
                     color);
}

void VisionOverlay_DrawBinaryContour(const uint8 *binary_image,
                                     int min_col, int max_col,
                                     int min_row, int max_row,
                                     uint16 color, uint16 image_y_offset)
{
    int row = 0;
    int col = 0;

    if(binary_image == NULL)
    {
        return;
    }

    if(min_col < 0) min_col = 0;
    if(max_col >= (int)VISION_RUNTIME_FINE_WIDTH)
        max_col = VISION_RUNTIME_FINE_WIDTH - 1;
    if(min_row < 0) min_row = 0;
    if(max_row >= (int)VISION_RUNTIME_FINE_HEIGHT)
        max_row = VISION_RUNTIME_FINE_HEIGHT - 1;
    if(min_col > max_col || min_row > max_row)
    {
        return;
    }

    for(row = min_row; row <= max_row; row++)
    {
        for(col = min_col; col <= max_col; col++)
        {
            int dr = 0;
            int dc = 0;
            uint8 is_boundary = 0U;

            if(!VisionOverlay_GetBinaryPixel(binary_image, col, row))
            {
                continue;
            }

            for(dr = -1; dr <= 1 && !is_boundary; dr++)
            {
                for(dc = -1; dc <= 1; dc++)
                {
                    int next_row = row + dr;
                    int next_col = col + dc;

                    if(dr == 0 && dc == 0)
                    {
                        continue;
                    }
                    if(next_row < 0 ||
                       next_row >= (int)VISION_RUNTIME_FINE_HEIGHT ||
                       next_col < 0 ||
                       next_col >= (int)VISION_RUNTIME_FINE_WIDTH ||
                       !VisionOverlay_GetBinaryPixel(binary_image,
                                                     next_col, next_row))
                    {
                        is_boundary = 1U;
                        break;
                    }
                }
            }

            if(is_boundary)
            {
                ips200_draw_point((uint16)(col / 2),
                                  (uint16)(row / 2 + (int)image_y_offset),
                                  color);
            }
        }
    }
}

void VisionOverlay_DrawWorldPoint4x4(float world_x, float world_y, uint16 color, uint16 image_y_offset)
{
    int img_x = 0;
    int img_y = 0;

    if(VisionOverlay_WorldToImage(world_x, world_y, &img_x, &img_y))
    {
        VisionOverlay_DrawImagePoint4x4(img_x, img_y, color, image_y_offset);
    }
}

void VisionOverlay_DrawWorldLine(float world_x0, float world_y0,
                                 float world_x1, float world_y1,
                                 uint16 color, uint16 image_y_offset)
{
    int img_x0 = 0;
    int img_y0 = 0;
    int img_x1 = 0;
    int img_y1 = 0;

    if(!VisionOverlay_WorldLineToImage(world_x0, world_y0, world_x1, world_y1,
                                       &img_x0, &img_y0, &img_x1, &img_y1))
    {
        return;
    }

    ips200_draw_line((uint16)img_x0,
                     (uint16)(img_y0 + (int)image_y_offset),
                     (uint16)img_x1,
                     (uint16)(img_y1 + (int)image_y_offset),
                     color);
}
