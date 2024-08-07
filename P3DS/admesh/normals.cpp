/*  ADMesh -- process triangulated solid meshes
 *  Copyright (C) 1995, 1996  Anthony D. Martin <amartin@engr.csulb.edu>
 *  Copyright (C) 2013, 2014  several contributors, see AUTHORS
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 *  Questions, comments, suggestions, etc to
 *           https://github.com/admesh/admesh/issues
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "stl.h"

static void stl_reverse_facet(stl_file *stl, int facet_num);
static void stl_reverse_vector(float v[]);
int stl_check_normal_vector(stl_file *stl, int facet_num, int normal_fix_flag);

extern float get_area(stl_facet *facet);
extern void stl_remove_facet(stl_file *stl, int facet_number);
bool stl_is_two_point_equal(stl_vertex point_1, stl_vertex point_2);

bool stl_is_two_point_equal(stl_vertex point_1, stl_vertex point_2)
{
	if ((fabs(point_1.x - point_2.x) < 0.000001)
		&& (fabs(point_1.y - point_2.y) < 0.000001)
		&& (fabs(point_1.z - point_2.z) < 0.000001))
	{
		return true;
	}
	else
	{
		return false;
	}
}

static void
stl_reverse_facet(stl_file *stl, int facet_num) {
	stl_vertex tmp_vertex;
	/*  int tmp_neighbor;*/
	int neighbor[3];
	int vnot[3];

	stl->stats.facets_reversed += 1;

	neighbor[0] = stl->neighbors_start[facet_num].neighbor[0];
	neighbor[1] = stl->neighbors_start[facet_num].neighbor[1];
	neighbor[2] = stl->neighbors_start[facet_num].neighbor[2];
	vnot[0] = stl->neighbors_start[facet_num].which_vertex_not[0];
	vnot[1] = stl->neighbors_start[facet_num].which_vertex_not[1];
	vnot[2] = stl->neighbors_start[facet_num].which_vertex_not[2];

	/* reverse the facet */
	tmp_vertex = stl->facet_start[facet_num].vertex[0];
	stl->facet_start[facet_num].vertex[0] =
		stl->facet_start[facet_num].vertex[1];
	stl->facet_start[facet_num].vertex[1] = tmp_vertex;

	/* fix the vnots of the neighboring facets */
	if (neighbor[0] != -1)
		stl->neighbors_start[neighbor[0]].which_vertex_not[(vnot[0] + 1) % 3] =
		(stl->neighbors_start[neighbor[0]].
			which_vertex_not[(vnot[0] + 1) % 3] + 3) % 6;
	if (neighbor[1] != -1)
		stl->neighbors_start[neighbor[1]].which_vertex_not[(vnot[1] + 1) % 3] =
		(stl->neighbors_start[neighbor[1]].
			which_vertex_not[(vnot[1] + 1) % 3] + 4) % 6;
	if (neighbor[2] != -1)
		stl->neighbors_start[neighbor[2]].which_vertex_not[(vnot[2] + 1) % 3] =
		(stl->neighbors_start[neighbor[2]].
			which_vertex_not[(vnot[2] + 1) % 3] + 2) % 6;

	/* swap the neighbors of the facet that is being reversed */
	stl->neighbors_start[facet_num].neighbor[1] = neighbor[2];
	stl->neighbors_start[facet_num].neighbor[2] = neighbor[1];

	/* swap the vnots of the facet that is being reversed */
	stl->neighbors_start[facet_num].which_vertex_not[1] = vnot[2];
	stl->neighbors_start[facet_num].which_vertex_not[2] = vnot[1];

	/* reverse the values of the vnots of the facet that is being reversed */
	stl->neighbors_start[facet_num].which_vertex_not[0] =
		(stl->neighbors_start[facet_num].which_vertex_not[0] + 3) % 6;
	stl->neighbors_start[facet_num].which_vertex_not[1] =
		(stl->neighbors_start[facet_num].which_vertex_not[1] + 3) % 6;
	stl->neighbors_start[facet_num].which_vertex_not[2] =
		(stl->neighbors_start[facet_num].which_vertex_not[2] + 3) % 6;
}

void
stl_fix_normal_directions(stl_file *stl, bool auto_repair)
{
	if (auto_repair)
	{
		LOGINFO("repairing... 开始进行法向方向错误修复。");
	}
	else
	{
		LOGINFO("checking... 开始进行法向方向错误检查。");
	}
	char *norm_sw;
	char *calculated_volume;
	/*  int edge_num;*/
	/*  int vnot;*/
	int checked = 0;
	int facet_num;
	//int check_result;
	/*  int next_facet;*/
	int i;
	int j;
	struct stl_normal
	{
		int               facet_num;
		struct stl_normal *next;
	};
	struct stl_normal *head;
	struct stl_normal *tail;
	struct stl_normal *newn;
	struct stl_normal *temp;

	//add by kong
	int* reversed_ids;
	int reversed_count = 0;
	int* shell_ids;
	int shell_ids_count = 0;
	int id;
	int force_exit = 0;
	int* all_shell_ids;
	int all_shell_ids_count = 0;

	LOGINFO("stl->error %d || stl->stats.unable_repair %d", stl->error, stl->stats.unable_repair);
	if (stl->error || stl->stats.unable_repair) return;

	/* Initialize linked list. */
	head = (struct stl_normal*)malloc(sizeof(struct stl_normal));
	if (head == NULL) LOGINFO("stl_fix_normal_directions");
	tail = (struct stl_normal*)malloc(sizeof(struct stl_normal));
	if (tail == NULL) LOGINFO("stl_fix_normal_directions");
	head->next = tail;
	tail->next = tail;

	/* Initialize list that keeps track of already fixed facets. */
	norm_sw = (char*)calloc(stl->stats.number_of_facets, sizeof(char));
	if (norm_sw == NULL) LOGINFO("stl_fix_normal_directions");

	/* Initialize list that keeps track of already calculated volume. */
	calculated_volume = (char*)calloc(stl->stats.number_of_facets, sizeof(char));
	if (calculated_volume == NULL) LOGINFO("stl_fix_normal_directions calculated_volume");

	//add by kong
	/* Initialize list that keeps track of reversed facets. */
	reversed_ids = (int*)calloc(stl->stats.number_of_facets, sizeof(int));
	if (reversed_ids == NULL) perror("stl_fix_normal_directions reversed_ids");

	shell_ids = (int*)calloc(stl->stats.number_of_facets, sizeof(int));
	if (shell_ids == NULL) perror("stl_fix_normal_directions shell_ids");

	all_shell_ids = (int*)calloc(stl->stats.number_of_facets, sizeof(int));
	if (all_shell_ids == NULL) perror("stl_fix_normal_directions shell_ids");

	facet_num = 0;
	/* If normal vector is not within tolerance and backwards:
	   Arbitrarily starts at face 0.  If this one is wrong, we're screwed.  Thankfully, the chances
	   of it being wrong randomly are low if most of the triangles are right: */
	//check_result = stl_check_normal_vector(stl, 0, 0);
	//LOGINFO("starts at face 0 check_result is %d", check_result);
	if (stl_check_normal_vector(stl, 0, 0) == 2)
	{
		if (auto_repair)
		{
			stl_reverse_facet(stl, 0);
			//add by kong
			reversed_ids[reversed_count++] = 0;
		}
	}

	/* Say that we've fixed this facet: */
	norm_sw[facet_num] = 1;
	checked++;

	for (;;)
	{
		/* Add neighbors_to_list.
		   Add unconnected neighbors to the list:a  */
		for (j = 0; j < 3; j++)
		{
			/* Reverse the neighboring facets if necessary. */
			if (stl->neighbors_start[facet_num].which_vertex_not[j] > 2)
			{
				/* If the facet has a neighbor that is -1, it means that edge isn't shared by another facet */
				if (stl->neighbors_start[facet_num].neighbor[j] != -1)
				{
					if (stl->neighbors_start[facet_num].is_invalid[j] != true && auto_repair)
					{
						if (norm_sw[stl->neighbors_start[facet_num].neighbor[j]] == 1)
						{
							LOGINFO("repairing... 已翻转的面片再次发现法向错误，面片 ID %d。", stl->neighbors_start[facet_num].neighbor[j]);
							/* trying to modify a facet already marked as fixed, revert all changes made until now and exit  */
							for (id = reversed_count - 1; id >= 0; --id)
							{
								stl_reverse_facet(stl, reversed_ids[id]);
							}
							force_exit = 1;
							LOGINFO("repairing... 法向错误翻转回到自身，发现无法修复模型，强行退出。");
							break;
						}
						else
						{
							stl_reverse_facet(stl, stl->neighbors_start[facet_num].neighbor[j]);
							reversed_ids[reversed_count++] = stl->neighbors_start[facet_num].neighbor[j];
						}
					}
				}
			}
			/* If this edge of the facet is connected: */
			if (stl->neighbors_start[facet_num].neighbor[j] != -1)
			{
				/* If we haven't fixed this facet yet, add it to the list: */
				if (norm_sw[stl->neighbors_start[facet_num].neighbor[j]] != 1)
				{
					/* Add node to beginning of list. */
					newn = (struct stl_normal*)malloc(sizeof(struct stl_normal));
					if (newn == NULL) LOGINFO("stl_fix_normal_directions");
					newn->facet_num = stl->neighbors_start[facet_num].neighbor[j];
					newn->next = head->next;
					head->next = newn;
				}
			}
		}

		//add by kong
		/* an error occourred, quit the for loop and exit */
		if (force_exit && auto_repair)
		{
			//stl->stats.number_of_parts += 1;
			stl->stats.unable_repair = true;
			break;
		}
		/* Get next facet to fix from top of list. */
		if (head->next != tail)
		{
			facet_num = head->next->facet_num;
			/* If facet is in list mutiple times */
			if (norm_sw[facet_num] != 1)
			{
				norm_sw[facet_num] = 1; /* Record this one as being fixed. */
				checked++;
			}
			temp = head->next;	/* Delete this facet from the list. */
			head->next = head->next->next;
			free(temp);
		}
		else
		{	/* if we ran out of facets to fix: */
			/* All of the facets in this part have been fixed. */
			
			/*calculate volume.*/
			long i;
			stl_vertex p0;
			stl_vertex p;
			stl_vertex n;
			float height;
			float area;
			float volume = 0.0;
			float facet_area = 0.0;

			/* Choose a point, any point as the reference */
			p0.x = stl->facet_start[facet_num].vertex[0].x;
			p0.y = stl->facet_start[facet_num].vertex[0].y;
			p0.z = stl->facet_start[facet_num].vertex[0].z;

			shell_ids_count = 0;
			for (i = 0; i < stl->stats.number_of_facets; i++) {

				if ((norm_sw[i] == 1)&&(calculated_volume[i] != 1))
				{
					p.x = stl->facet_start[i].vertex[0].x - p0.x;
					p.y = stl->facet_start[i].vertex[0].y - p0.y;
					p.z = stl->facet_start[i].vertex[0].z - p0.z;
					/* Do dot product to get distance from point to plane */
					n = stl->facet_start[i].normal;
					height = (n.x * p.x) + (n.y * p.y) + (n.z * p.z);
					area = get_area(&stl->facet_start[i]);
					facet_area += area;
					volume += (area * height) / 3.0;
					calculated_volume[i] = 1;
					shell_ids[shell_ids_count++] = i;
				}
			}
			bool can_delete = false;
			LOGINFO("checking... 发现多壳体，此壳体面片数 %d，体积 %f，面积 %f。", shell_ids_count, volume, facet_area);
			if (volume < 0)
			{
				can_delete = true;
			}
			else if ((fabs(volume) < 0.000001) && (fabs(facet_area) < 0.000001))
			{
				can_delete = true;
			}
			else if ((fabs(volume) < 0.000001) && (facet_area < 0.1) && (shell_ids_count != 2))
			{
				can_delete = true;
			}
			else
			{
				can_delete = false;
			}
			if (can_delete)
			{
				for (int i = 0; i < shell_ids_count; i++)
				{
					all_shell_ids[all_shell_ids_count++] = shell_ids[i];
				}
				stl->repair_stats.noise_shells += 1;
				if (auto_repair)
				{
					LOGINFO("repairing... 发现多壳体错误，此错误壳体面片数 %d，体积 %f，面积 %f。待删除。", shell_ids_count, volume, facet_area);
				}
				else 
				{
					LOGINFO("checking... 发现多壳体错误，此错误壳体面片数 %d，体积 %f，面积 %f。", shell_ids_count, volume, facet_area);
				}
			}
			else
			{
				// 检查 + 修复，壳体数量计算为 2 
				if (auto_repair)
				{
					stl->stats.number_of_parts += 1;
				}				
			}
			if (checked >= stl->stats.number_of_facets)
			{
				/* All of the facets have been checked.  Bail out. */
				break;
			}
			else
			{
				/* There is another part here.  Find it and continue. */
				for (i = 0; i < stl->stats.number_of_facets; i++)
				{
					if (norm_sw[i] == 0)
					{
						/* This is the first facet of the next part. */
						facet_num = i;
						if (stl_check_normal_vector(stl, i, 0) == 2 && auto_repair)
						{
							stl_reverse_facet(stl, i);
							//add by kong
							reversed_ids[reversed_count++] = i;
						}
						norm_sw[facet_num] = 1;
						checked++;
						break;
					}
				}
			}
		}
	}
	if (auto_repair)
	{
		LOGINFO("repairing... 法向修复过程中共发生翻转面片数量 %d。", reversed_count);
	}
	if (all_shell_ids_count != 0 && auto_repair)
	{
		for (int i = 0; i < all_shell_ids_count; i++)
		{
			stl_remove_degenerate(stl, all_shell_ids[i]);
		}
		LOGINFO("repairing... 共删除多壳体错误面片数量 %d。", all_shell_ids_count);
		//拓扑重构
		stl_repair(stl, 0, 1, 1, 0, 1, 0, 1, 2, 1, 0, 0, 0, 0, 0);
	}
	free(head);
	free(tail);
	free(reversed_ids);
	free(norm_sw);
	free(calculated_volume);
	free(shell_ids);
	free(all_shell_ids);
}

int
stl_check_normal_vector(stl_file *stl, int facet_num, int normal_fix_flag) {
	/* Returns 0 if the normal is within tolerance */
	/* Returns 1 if the normal is not within tolerance, but direction is OK */
	/* Returns 2 if the normal is not within tolerance and backwards */
	/* Returns 4 if the status is unknown. */

  float normal[3];
  float test_norm[3];
  stl_facet *facet;

  facet = &stl->facet_start[facet_num];

  stl_calculate_normal(normal, facet);
  stl_normalize_vector(normal);

  if(   (ABS(normal[0] - facet->normal.x) < 0.001)
        && (ABS(normal[1] - facet->normal.y) < 0.001)
        && (ABS(normal[2] - facet->normal.z) < 0.001)) {
    /* It is not really necessary to change the values here */
    /* but just for consistency, I will. */
    facet->normal.x = normal[0];
    facet->normal.y = normal[1];
    facet->normal.z = normal[2];
    return 0;
  }

  test_norm[0] = facet->normal.x;
  test_norm[1] = facet->normal.y;
  test_norm[2] = facet->normal.z;
  stl_normalize_vector(test_norm);

  if(   (ABS(normal[0] - test_norm[0]) < 0.001)
        && (ABS(normal[1] - test_norm[1]) < 0.001)
        && (ABS(normal[2] - test_norm[2]) < 0.001)) {
    if(normal_fix_flag) {
      facet->normal.x = normal[0];
      facet->normal.y = normal[1];
      facet->normal.z = normal[2];
      stl->stats.normals_fixed += 1;
    }
    return 1;
  }

  stl_reverse_vector(test_norm);
  if(   (ABS(normal[0] - test_norm[0]) < 0.001)
        && (ABS(normal[1] - test_norm[1]) < 0.001)
        && (ABS(normal[2] - test_norm[2]) < 0.001)) {
    /* Facet is backwards. */
    if(normal_fix_flag) {
      facet->normal.x = normal[0];
      facet->normal.y = normal[1];
      facet->normal.z = normal[2];
      stl->stats.normals_fixed += 1;
    }
    return 2;
  }

  if(normal_fix_flag) {
    facet->normal.x = normal[0];
    facet->normal.y = normal[1];
    facet->normal.z = normal[2];
    stl->stats.normals_fixed += 1;
  }
  return 4;
}

static void
stl_reverse_vector(float v[]) {
  v[0] *= -1;
  v[1] *= -1;
  v[2] *= -1;
}


void
stl_calculate_normal(float normal[], stl_facet *facet) {
  float v1[3];
  float v2[3];

  v1[0] = facet->vertex[1].x - facet->vertex[0].x;
  v1[1] = facet->vertex[1].y - facet->vertex[0].y;
  v1[2] = facet->vertex[1].z - facet->vertex[0].z;
  v2[0] = facet->vertex[2].x - facet->vertex[0].x;
  v2[1] = facet->vertex[2].y - facet->vertex[0].y;
  v2[2] = facet->vertex[2].z - facet->vertex[0].z;

  normal[0] = (float)((double)v1[1] * (double)v2[2]) - ((double)v1[2] * (double)v2[1]);
  normal[1] = (float)((double)v1[2] * (double)v2[0]) - ((double)v1[0] * (double)v2[2]);
  normal[2] = (float)((double)v1[0] * (double)v2[1]) - ((double)v1[1] * (double)v2[0]);
}

void stl_normalize_vector(float v[]) {
  double length;
  double factor;
  float min_normal_length;

  length = sqrt((double)v[0] * (double)v[0] + (double)v[1] * (double)v[1] + (double)v[2] * (double)v[2]);
  min_normal_length = 0.000000000001;
  if(length < min_normal_length) {
    v[0] = 0.0;
    v[1] = 0.0;
    v[2] = 0.0;
    return;
  }
  factor = 1.0 / length;
  v[0] *= factor;
  v[1] *= factor;
  v[2] *= factor;
}

void
stl_fix_normal_values(stl_file *stl, bool auto_repair) {
	
	if (auto_repair)
	{
		LOGINFO("repairing... 开始矫正面片法向值，使读取值与计算值相等。");
	}
	else
	{
		LOGINFO("checking... 开始检查面片法向值，使读取值与计算值相等。");
	}
	
  int i;

  LOGINFO("stl->error %d || stl->stats.unable_repair %d", stl->error, stl->stats.unable_repair);
  if (stl->error || stl->stats.unable_repair) return;

  if (auto_repair == false)
  {
	  return;
  }

  for(i = 0; i < stl->stats.number_of_facets; i++) {
    stl_check_normal_vector(stl, i, 1);
  }
}

void
stl_reverse_all_facets(stl_file *stl) {
  int i;
  float normal[3];

  if (stl->error) return;

  for(i = 0; i < stl->stats.number_of_facets; i++) {
    stl_reverse_facet(stl, i);
    stl_calculate_normal(normal, &stl->facet_start[i]);
    stl_normalize_vector(normal);
    stl->facet_start[i].normal.x = normal[0];
    stl->facet_start[i].normal.y = normal[1];
    stl->facet_start[i].normal.z = normal[2];
  }
}

double 
stl_calculate_faceangle(stl_facet *facet)
{
	float f_normal[3];
	stl_calculate_normal(f_normal, facet);
	stl_normalize_vector(f_normal);
	facet->normal.x = f_normal[0];
	facet->normal.y = f_normal[1];
	facet->normal.z = f_normal[2];
	return (acos(-facet->normal.z) * 180.0 / PI);
}

