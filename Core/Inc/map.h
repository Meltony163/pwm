/*
 * map.h
 *
 *  Created on: Oct 29, 2024
 *      Author: moame
 */

#ifndef INC_MAP_H_
#define INC_MAP_H_

u32 map(u32 x, u32 in_min, u32 in_max, u32 out_min, u32 out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#endif /* INC_MAP_H_ */
