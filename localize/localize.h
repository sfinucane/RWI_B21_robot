#ifndef LOCALIZE_INCLUDE
#define LOCALIZE_INCLUDE

#ifdef __cplusplus
extern "C" {
#endif

/* Sends the position to the simulator and localize. Tcx and both
 * modules must already be connected. */
void
setRobotPosition( float x, float y, float rot);


/* Robot coordinates must be given in original corrdinates as provided by
   the base report.
   Map coordinates must be given in world coordinates where rot is given in
   rad and for a rotation of zero the robot faces into x-direction.
   */

void
robotCoordinates2MapCoordinates( float robX, float robY, float robRot,
				 float corrX, float corrY, float corrRot, int corrType,
				 float* mapX, float* mapY, float* mapRot);

void
mapCoordinates2RobotCoordinates( float mapX, float mapY, float mapRot,
				 float corrX, float corrY, float corrRot, int corrType,
				 float* robX, float* robY, float* robRot);

#ifdef __cplusplus
}
#endif

#endif







