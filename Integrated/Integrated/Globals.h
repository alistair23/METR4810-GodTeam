#ifndef _INCLUDED_GLOBALS_H
#define _INCLUDED_GLOBALS_H

#define CIRCLE_MARK_DIAMETER 0.05	// Diameter of finish line/checkpoint circles in metres
#define CIRCLE_MARK_DIST 0.28		// Distance between circles
#define ROAD_WIDTH 0.2				// Road width in metres
#define M_PER_PIX 0.004				// Metres per pixel, used for internal computation
#define SOURCE_M_PER_PIX 0.001		// Metres per pixel for source tile images
#define FINISH_SOURCE_MARKER_X 50	// X coordinate of marker pair centre in pixels (measured from source image)
#define FINISH_SOURCE_MARKER_Y 250	// Y coordinate of marker pair centre in pixels (measured from source image)
#define TILE_M_LENGTH	0.5			// Side length of (square) tile in metres
#define GO_SIGNAL_DIAMETER_PIX 0.04 / M_PER_PIX	
#define GO_SIGNAL_MAX_DIST_TO_FINISH_LINE_PIX 600

#define DEFAULT_CAR_LENGTH_PIX 0.15 / M_PER_PIX		// Default car dimensions in pixels
#define DEFAULT_CAR_WIDTH_PIX 0.075 / M_PER_PIX
#define DEFAULT_CAR_HEIGHT_PIX 0.05 / M_PER_PIX
#define SIDE_CLEARANCE_PIX 0.01 / M_PER_PIX			// For collision testing between cars, in pixels
#define FRONT_CLEARANCE_PIX 0.05 / M_PER_PIX

// Stuff for our setup-time marker in metres
#define OUR_CENTRE_DIAMETER_BIG 0.065
#define OUR_CENTRE_DIAMETER_SMALL 0.035
#define OUR_DIAMTER 0.035
#define OUR_SQUARE_SIDE 0.13

#define MY_CAR_MARKER_BIGGEST 0.075
#define MY_CAR_MARKER_SMALLEST 0.045

#endif