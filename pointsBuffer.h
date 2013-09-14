#ifndef POINTSBUFFER_H
#define POINTSBUFFER_H

#include <stm32f4xx.h>

/*
 * @brief Struct to hold trajectory data at a single point in space
 */
typedef volatile struct {
	float X;	/*<< X coordinate in mm*/
	float Y;	/*<< Y coordinate in mm*/
	float O;	/*<< Orientation angle in radians */
	float V;	/*<< Linear speed in m/s */
	float W;	/*<< Rotational speed in rad/s */
} TrajectoryPoint_Struct;

/*
 * @brief Access next loaded point
 * @return Const pointer to a location where trajectory point data is stored. Returns NULL if there is no points left.
 */
const TrajectoryPoint_Struct* const TBgetNextPoint();
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
 * @brief Resets buffer to its default state. All data is discarded and lost.
 * @return Number of trajectory points that were discarded.
 */
void TBresetBuffer();
/*
 * @brief Function that must be called in response to DMA transfer being finished (ISR handler)
 */
void TBDMATransferCompletedSlot();

#endif /* POINTSBUFFER_H */
