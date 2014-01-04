#ifndef POINTSBUFFER_H
#define POINTSBUFFER_H

#include <stm32f4xx.h>
#include <stdbool.h>

/*
 * @brief Struct to hold trajectory data at a single point in space
 */
typedef volatile struct {
	float X;	/*<< X coordinate in metres */
	float Y;	/*<< Y coordinate in metres */
	float O;	/*<< Orientation angle in radians */
	float V;	/*<< Linear speed in m/s */
	float W;	/*<< Rotational speed in rad/s */
} TrajectoryPoint_Struct;

/*
 * @brief Access next loaded point
 * @param point Structure to which new point data will be loaded
 * @return False if no points in buffer and point is invalid, true otherwise
 */
bool TBgetNextPoint(TrajectoryPoint_Struct* point);
/*
 * @brief Initiates DMA transfer to load new trajectory points.
 * @param num Requested number of points to load
 * @return Number of points that will be really loaded.
 *
 * If there is no enough space for all new points requested, the highest possible number of points will be loaded
 * There is no possibility to overwrite points that was not accessed by getNextPoint
 */
uint16_t TBloadNewPoints(uint16_t num);
/*
 * @brief Return number of points ready to be read from buffer
 */
uint16_t TBgetAvailablePoints();
/*
 * @brief Returns percent of free/used space in buffer
 * @return (0.0 - 1.0) - percent of free space in the buffer
 */
float TBgetFreeSpace();
float TBgetUsedSpace();
/*
 * @brief Get size of the buffer
 * @return Overall size of the buffer, number of points that will fit in
 */
uint16_t TBgetSize();
/*
 * @brief Resets buffer to its default state. All data is discarded and lost.
 * @return Number of trajectory points that were discarded.
 */
void TBresetBuffer();

#endif /* POINTSBUFFER_H */
