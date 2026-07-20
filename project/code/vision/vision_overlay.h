#ifndef VISION_OVERLAY_H_
#define VISION_OVERLAY_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8 VisionOverlay_WorldToImage(float world_x, float world_y, int *img_x, int *img_y);
uint8 VisionOverlay_WorldLineToImage(float world_x0, float world_y0,
                                     float world_x1, float world_y1,
                                     int *img_x0, int *img_y0,
                                     int *img_x1, int *img_y1);

void VisionOverlay_DrawImagePoint4x4(int img_x, int img_y, uint16 color, uint16 image_y_offset);
void VisionOverlay_DrawFineImagePoint4x4(int img_x, int img_y, uint16 color, uint16 image_y_offset);
void VisionOverlay_DrawFineImageLine(int img_x0, int img_y0,
                                     int img_x1, int img_y1,
                                     uint16 color, uint16 image_y_offset);
void VisionOverlay_DrawBinaryContour(const uint8 *binary_image,
                                     int min_col, int max_col,
                                     int min_row, int max_row,
                                     uint16 color, uint16 image_y_offset);
void VisionOverlay_DrawWorldPoint4x4(float world_x, float world_y, uint16 color, uint16 image_y_offset);
void VisionOverlay_DrawWorldLine(float world_x0, float world_y0,
                                 float world_x1, float world_y1,
                                 uint16 color, uint16 image_y_offset);

#ifdef __cplusplus
}
#endif

#endif
