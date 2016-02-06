/*
  motion_control.c - high level interface for issuing motion commands
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011 Sungeun K. Jeon
  
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
 
// mc_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], feedrate*feedmultiply/60/100.0, active_extruder);
void mc_line(float curx, float cury,float x, float y, float z, float e, float feed_rate, uint8_t extruder)
{
	// Break accel and decel phases into 0.2 - 0.5 mm segments, let planner handle acceleration and intensity calculations
	// _ _ _ _ _ _ _ _ _______________________________ _ _ _ _ _ _ _ <-- block lengths
	const float MOVELENGTH = 0.2;
	uint16_t i;
	// How long is the original gcode move?
	float command_length;
	command_length = sqrt(pow(curx+x, 2) + pow(cury+y, 2)) ;
	// How long (in mm) does acceleration take?
	// feed_rate mm/sec, acceleration mm/sec^2 
	float accel_max_time ;
	accel_max_time = feed_rate / max_acceleration_units_per_sq_second[0]; // in ConfigurationStore.cpp, defined where?
	// feed / accel  =  sec, time to accelerate
	float accel_dist;
	accel_dist = ((0.5*feed_rate) * accel_max_time);
	// 1/2 *(feed_rate-0)  =  avg velocity during acceleration (assuming start from 0 mm/sec, usually we'll be in motion already though, but this way every case gets covered.)
	// avg velocity * time to accel  =  distance traveled during acceleration
	int accelmoves, totalmoves; // number of tiny moves that make up the accel phase, ...that would make up the whole move if they were all equally long
	accelmoves = floor(accel_dist / MOVELENGTH);
	totalmoves = floor(command_length / MOVELENGTH);
	// How to break accel blocks into suitable lengths?
	// get x,y vector that moves 0.2 mm 
	float xvector, yvector;
	xvector = (x-curx) / totalmoves;
	yvector = (y-cury) / totalmoves;
	
	// Can accel distance fit twice in the whole gcode command's length?
	if (command_length > accel_dist*2){ // More than 2 maximum accelerations fit in this G1 command
		// Break in accel blocks, main block, and decel blocks
		for (i = 1; i<=accelmoves; i++){ // Advance 0.2mm along the G1 move's path per block
			curx = curx+xvector;
			cury = cury+yvector;
			plan_buffer_line(curx, cury, z, e, feed_rate, extruder);
		}
		// main block
		curx = x - (accelmoves * xvector);
		cury = y - (accelmoves * yvector);
		plan_buffer_line(curx, cury, z, e, feed_rate, extruder);
		for (i = 1; i<=accelmoves; i++){ // Advance 0.2mm along the G1 move's path per block
			curx = curx+xvector;
			cury = cury+yvector;
			plan_buffer_line(curx, cury, z, e, feed_rate, extruder);
		}
	} else { // Can't fit two accelerations in the move
		// Can only fit short blocks.
		for (i = 1; i<=accelmoves; i++){ // Advance 0.2mm along the G1 move's path per block
			curx = curx+xvector;
			cury = cury+yvector;
			plan_buffer_line(curx, cury, z, e, feed_rate, extruder);
		}
	}
	// check that we have reached the destination, else make a tiny move to do so
	
	
	
  
	// For each block, calculate end position x,y,z

	plan_buffer_line(x, y, z, e, feed_rate, extruder);
}


// The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
// segment is configured in settings.mm_per_arc_segment.  
void mc_arc(float *position, float *target, float *offset, uint8_t axis_0, uint8_t axis_1, 
  uint8_t axis_linear, float feed_rate, float radius, uint8_t isclockwise, uint8_t extruder)
{      
  //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
  //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float linear_travel = target[axis_linear] - position[axis_linear];
  float extruder_travel = target[E_AXIS] - position[E_AXIS];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;
  
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (angular_travel < 0) { angular_travel += 2*M_PI; }
  if (isclockwise) { angular_travel -= 2*M_PI; }
  
  float millimeters_of_travel = hypot(angular_travel*radius, fabs(linear_travel));
  if (millimeters_of_travel < 0.001) { return; }
  uint16_t segments = floor(millimeters_of_travel/MM_PER_ARC_SEGMENT);
  if(segments == 0) segments = 1;
  
  /*  
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
    // all segments.
    if (invert_feed_rate) { feed_rate *= segments; }
  */
  float theta_per_segment = angular_travel/segments;
  float linear_per_segment = linear_travel/segments;
  float extruder_per_segment = extruder_travel/segments;
  
  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
         r_T = [cos(phi) -sin(phi);
                sin(phi)  cos(phi] * r ;
     
     For arc generation, the center of the circle is the axis of rotation and the radius vector is 
     defined from the circle center to the initial position. Each line segment is formed by successive
     vector rotations. This requires only two cos() and sin() computations to form the rotation
     matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     all double numbers are single precision on the Arduino. (True double precision will not have
     round off issues for CNC applications.) Single precision error can accumulate to be greater than
     tool precision in some cases. Therefore, arc path correction is implemented. 

     Small angle approximation may be used to reduce computation overhead further. This approximation
     holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
     theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for 
     numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     issue for CNC machines with the single precision Arduino calculations.
     
     This approximation also allows mc_arc to immediately insert a line segment into the planner 
     without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead. 
     This is important when there are successive arc motions. 
  */
  // Vector rotation matrix values
  float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;
  
  float arc_target[4];
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;

  // Initialize the linear axis
  arc_target[axis_linear] = position[axis_linear];
  
  // Initialize the extruder axis
  arc_target[E_AXIS] = position[E_AXIS];

  for (i = 1; i<segments; i++) { // Increment (segments-1)
    
    if (count < N_ARC_CORRECTION) {
      // Apply vector rotation matrix 
      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    } else {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti = cos(i*theta_per_segment);
      sin_Ti = sin(i*theta_per_segment);
      r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
      r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[axis_0] = center_axis0 + r_axis0;
    arc_target[axis_1] = center_axis1 + r_axis1;
    arc_target[axis_linear] += linear_per_segment;
    arc_target[E_AXIS] += extruder_per_segment;

    clamp_to_software_endstops(arc_target);
    plan_buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], arc_target[Z_AXIS], arc_target[E_AXIS], feed_rate, extruder);
    
  }
  // Ensure last segment arrives at target location.
  plan_buffer_line(target[X_AXIS], target[Y_AXIS], target[Z_AXIS], target[E_AXIS], feed_rate, extruder);

  //   plan_set_acceleration_manager_enabled(acceleration_manager_was_enabled);
}

