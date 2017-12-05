#include <stdio.h>
#include <stdlib.h>

/**
 * For face formular
 */
typedef struct mdl_face_s
{
	/**
	 * Parameters' formular
	 */
	float ap; float bp; float cp; float dp;

	/*
	 * point-normal formular
	 * */
	float pt[3];
	float norm[3];
} mdl_face_t;

/**
 * World large open-box model
 */
typedef struct mdl_world_s
{
	mdl_face_t bottom_face;

	/**
	 * left face consists of smallest x-coords
	 * while back face smallest z-coords
	 */
	mdl_face_t left_face;
	mdl_face_t right_face;
	mdl_face_t front_face;
	mdl_face_t back_face;

} mdl_world_t;
