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

static void stl_match_neighbors_exact(stl_file *stl,
                                      stl_hash_edge *edge_a, stl_hash_edge *edge_b);
static void stl_match_neighbors_nearby(stl_file *stl,
                                       stl_hash_edge *edge_a, stl_hash_edge *edge_b);
static void stl_record_neighbors(stl_file *stl,
                                 stl_hash_edge *edge_a, stl_hash_edge *edge_b);
static void stl_initialize_facet_check_exact(stl_file *stl);
static void stl_initialize_facet_check_nearby(stl_file *stl);
static void stl_load_edge_exact(stl_file *stl, stl_hash_edge *edge,
                                stl_vertex *a, stl_vertex *b);
static int stl_load_edge_nearby(stl_file *stl, stl_hash_edge *edge,
                                stl_vertex *a, stl_vertex *b, float tolerance);
static void insert_hash_edge(stl_file *stl, stl_hash_edge edge,
                             void (*match_neighbors)(stl_file *stl,
                                                     stl_hash_edge *edge_a, stl_hash_edge *edge_b));
static int stl_get_hash_for_edge(int M, stl_hash_edge *edge);
static int stl_compare_function(stl_hash_edge *edge_a, stl_hash_edge *edge_b);
static void stl_free_edges(stl_file *stl);
void stl_remove_facet(stl_file *stl, int facet_number);
static void stl_change_vertices(stl_file *stl, int facet_num, int vnot,
                                stl_vertex new_vertex);
static void stl_which_vertices_to_change(stl_file *stl, stl_hash_edge *edge_a,
                                         stl_hash_edge *edge_b, int *facet1, int *vertex1,
                                         int *facet2, int *vertex2,
                                         stl_vertex *new_vertex1, stl_vertex *new_vertex2);
extern int stl_check_normal_vector(stl_file *stl,
                                   int facet_num, int normal_fix_flag);
static void stl_update_connects_remove_1(stl_file *stl, int facet_num);

extern bool stl_is_two_point_equal(stl_vertex point_1, stl_vertex point_2);
static bool stl_is_two_facet_almost_coplane(stl_facet facet_1, stl_facet facet_2, float tolerance);
static bool stl_is_facet_too_thin(stl_facet facet, int multiple);
extern float get_area(stl_facet *facet);
static bool stl_is_degenerate_tiny_facet(stl_file *stl, int facet_num, float tolerance);
static bool stl_swap_facets_neighbor(stl_file *stl, int facet_num_a1, int facet_num_a2, int facet_num_b1, int facet_num_b2);
static bool stl_any_two_facet_area_equal(stl_file *stl, int *facet_ids);

static bool stl_is_degenerate_tiny_facet(stl_file *stl, int facet_num, float tolerance)
{
  stl_facet tiny_facet = stl->facet_start[facet_num];
  float tiny_facet_area = get_area(&tiny_facet);
  if (tiny_facet_area < tolerance)
  {
    return true;
  }
  else
  {
    return false;
  }
}

static bool stl_is_facet_too_thin(stl_facet facet, int multiple)
{
  float max_edge = 0;
  float min_edge = 0;
  float temp = 0;
  float edge_1 = sqrt(pow((facet.vertex[0].x - facet.vertex[1].x), 2) + pow((facet.vertex[0].y - facet.vertex[1].y), 2) + pow((facet.vertex[0].z - facet.vertex[1].z), 2));
  float edge_2 = sqrt(pow((facet.vertex[1].x - facet.vertex[2].x), 2) + pow((facet.vertex[1].y - facet.vertex[2].y), 2) + pow((facet.vertex[1].z - facet.vertex[2].z), 2));
  float edge_3 = sqrt(pow((facet.vertex[2].x - facet.vertex[0].x), 2) + pow((facet.vertex[2].y - facet.vertex[0].y), 2) + pow((facet.vertex[2].z - facet.vertex[0].z), 2));
  temp = (edge_1 > edge_2) ? edge_1 : edge_2;
  max_edge = (temp > edge_3) ? temp : edge_3;
  temp = (edge_1 < edge_2) ? edge_1 : edge_2;
  min_edge = (temp < edge_3) ? temp : edge_3;
  float x = max_edge / min_edge;

  if (max_edge / min_edge > multiple)
  {
    return true;
  }
  else
  {
    return false;
  }
}

static bool stl_is_two_facet_almost_coplane(stl_facet facet_1, stl_facet facet_2, float tolerance)
{
  if (fabs(facet_1.normal.x - facet_2.normal.x) < tolerance && fabs(facet_1.normal.y - facet_2.normal.y) < tolerance && fabs(facet_1.normal.z - facet_2.normal.z) < tolerance)
  {
    return true;
  }
  else if (fabs(fabs(facet_1.normal.x) - fabs(facet_2.normal.x)) < tolerance && fabs(fabs(facet_1.normal.y) - fabs(facet_2.normal.y)) < tolerance && fabs(fabs(facet_1.normal.z) - fabs(facet_2.normal.z)) < tolerance)
  {
    return true;
  }
  else
  {
    return false;
  }
}

static bool stl_any_two_facet_area_equal(stl_file *stl, int *facet_ids, int count)
{
  float first_facet_area = -1;
  for (int n = 0; n < count; n++)
  {
    int facet_num = facet_ids[n];
    stl_facet temp_facet_n = stl->facet_start[facet_num];
    float area = get_area(&temp_facet_n);
    if (first_facet_area == -1)
    {
      first_facet_area = area;
    }
    if (first_facet_area == area)
    {
      return true;
    }
  }
  return false;
}

void stl_fix_facets_shared_one_edge(stl_file *stl, bool auto_repair = true)
{
  LOGINFO("stl->error %d || stl->stats.unable_repair %d", stl->error, stl->stats.unable_repair);
  if (stl->error || stl->stats.unable_repair)
    return;

  if (auto_repair)
  {
    LOGINFO("repairing..");
  }
  else
  {
    LOGINFO("checking... ");
  }

  stl_facet error_facet;

  int *checked_facet_ids;
  int checked_facet_count = 0;

  // 法向错误边界
  stl_edge *normal_error_loop;
  int normal_error_count = 0;

  int more_two_facets_shared_one_edge_count = 0;

  int *many_shared_facet_ids;

  int *all_many_shared_facet_ids;
  int removed_count = 0;

  int *has_deleted_facet_ids;

  checked_facet_ids = (int *)calloc(stl->stats.number_of_facets, sizeof(int));
  if (checked_facet_ids == NULL)
    LOGINFO("stl_fix_normal_directions checked_facet_ids");

  normal_error_loop = (stl_edge *)calloc(stl->stats.number_of_facets, sizeof(stl_edge));
  if (normal_error_loop == NULL)
    LOGINFO("stl_fix_normal_directions normal_error_loop");

  many_shared_facet_ids = (int *)calloc(stl->stats.number_of_facets, sizeof(int));
  if (many_shared_facet_ids == NULL)
    LOGINFO("stl_fix_normal_directions many_shared_facet_ids");

  all_many_shared_facet_ids = (int *)calloc(stl->stats.number_of_facets, sizeof(int));
  if (all_many_shared_facet_ids == NULL)
    LOGINFO("stl_fix_normal_directions all_many_shared_facet_ids");

  has_deleted_facet_ids = (int *)calloc(stl->stats.number_of_facets, sizeof(int));
  if (has_deleted_facet_ids == NULL)
    LOGINFO("stl_fix_normal_directions has_deleted_facet_ids");

  bool is_neighbor_match_well = true;

  // 检查找出法向错误边
  for (int i = 0; i < stl->stats.number_of_facets; i++)
  {
    // 互为邻居的面片只找一个
    if (checked_facet_count != 0)
    {
      bool is_checked = false;
      for (int k = 0; k < checked_facet_count; k++)
      {
        if (i == checked_facet_ids[k])
        {
          is_checked = true;
        }
      }
      if (is_checked)
      {
        continue;
      }
    }
    for (int j = 0; j < 3; j++)
    {
      if (stl->neighbors_start[i].which_vertex_not[j] > 2)
      {
        if (stl->neighbors_start[i].neighbor[j] != -1)
        {
          checked_facet_ids[checked_facet_count++] = stl->neighbors_start[i].neighbor[j];
          error_facet = stl->facet_start[i];

          if (j == 0)
          {
            int tmp = normal_error_count++;
            normal_error_loop[tmp].p1 = error_facet.vertex[0];
            normal_error_loop[tmp].p2 = error_facet.vertex[1];
            normal_error_loop[tmp].facet_number = i;
          }
          else if (j == 1)
          {
            int tmp = normal_error_count++;
            normal_error_loop[tmp].p1 = error_facet.vertex[1];
            normal_error_loop[tmp].p2 = error_facet.vertex[2];
            normal_error_loop[tmp].facet_number = i;
          }
          else
          {
            int tmp = normal_error_count++;
            normal_error_loop[tmp].p1 = error_facet.vertex[2];
            normal_error_loop[tmp].p2 = error_facet.vertex[0];
            normal_error_loop[tmp].facet_number = i;
          }
        }
      }
      if (normal_error_count > stl->stats.number_of_facets * 2 + 1)
      {
        stl->stats.unable_repair = true;
        return;
      }
    }
  }
  for (int i = 0; i < normal_error_count; i++)
  {
    stl_edge error_edge = normal_error_loop[i];
    int count = 0;
    for (int j = 0; j < stl->stats.number_of_facets; j++)
    {
      stl_vertex p1 = stl->facet_start[j].vertex[0];
      stl_vertex p2 = stl->facet_start[j].vertex[1];
      stl_vertex p3 = stl->facet_start[j].vertex[2];
      if (stl_is_two_point_equal(error_edge.p1, p1) || stl_is_two_point_equal(error_edge.p1, p2) || stl_is_two_point_equal(error_edge.p1, p3))
      {
        if (stl_is_two_point_equal(error_edge.p2, p1) || stl_is_two_point_equal(error_edge.p2, p2) || stl_is_two_point_equal(error_edge.p2, p3))
        {
          many_shared_facet_ids[count++] = j;
        }
      }
    }
    if (count == 4)
    {
      if (stl_any_two_facet_area_equal(stl, many_shared_facet_ids, 4))
      {
        for (int n = 0; n < count; n++)
        {
          int facet_num = many_shared_facet_ids[n];
          for (int j = 0; j < 3; j++)
          {
            if (stl->neighbors_start[facet_num].which_vertex_not[j] > 2)
            {
              if (auto_repair)
              {
                stl->neighbors_start[facet_num].is_invalid[j] = true;
              }
            }
          }
        }
        stl->stats.no_need_fix_normal_direction = true;
        continue;
      }

      if (auto_repair)
      {
        // 拿到一组共边面片
      }
      else
      {
        // 拿到一组共边面片
      }
      more_two_facets_shared_one_edge_count++;
      for (int n = 0; n < count; n++)
      {
        int facet_num = many_shared_facet_ids[n];
        stl_facet temp_facet_n = stl->facet_start[facet_num];
        if (auto_repair)
        {
        }
        else
        {
        }
        int tmp = 0;
        int tmp_1 = 0;
        for (int m = 0; m < count; m++)
        {
          stl_facet temp_facet_m = stl->facet_start[many_shared_facet_ids[m]];
          if (stl_is_two_facet_almost_coplane(temp_facet_n, temp_facet_m, 0.01))
          {
            tmp++;
          }
          if (stl_is_degenerate_tiny_facet(stl, many_shared_facet_ids[m], 0.000001))
          {
            tmp_1++;
          }
        }
        if (auto_repair)
        {
        }
        else
        {
        }
        if (tmp == count)
        {
          for (int l = 0; l < count; l++)
          {
            if (removed_count != 0)
            {
              bool existed = false;
              for (int k = 0; k < removed_count; k++)
              {
                if (many_shared_facet_ids[l] == all_many_shared_facet_ids[k])
                {
                  existed = true;
                }
              }
              if (existed)
              {
                continue;
              }
              else
              {
                all_many_shared_facet_ids[removed_count++] = many_shared_facet_ids[l];
                if (auto_repair)
                {
                }
                else
                {
                }
              }
            }
            else
            {
              all_many_shared_facet_ids[removed_count++] = many_shared_facet_ids[l];
              if (auto_repair)
              {
              }
              else
              {
              }
            }
          }
          break;
        }
        else if (tmp_1 == count)
        {
          for (int l = 0; l < count; l++)
          {
            // stl_remove_facet(stl, many_shared_facet_ids[l]);
            if (auto_repair)
            {
            }
            else
            {
            }
          }
          break;
        }
        else
        {
          if (auto_repair)
          {
          }
          else
          {
          }
          for (int j = 0; j < 3; j++)
          {
            if (stl->neighbors_start[facet_num].which_vertex_not[j] > 2)
            {
              stl_facet temp_facet = stl->facet_start[stl->neighbors_start[facet_num].neighbor[j]];
              if (auto_repair)
              {
                stl->neighbors_start[facet_num].is_invalid[j] = true;
              }
              else
              {
              }
            }
          }
        }
      }

      // 匹配邻居
      int facet_num_a1 = -1;
      int facet_num_a2 = -1;
      int facet_num_b1 = -1;
      int facet_num_b2 = -1;
      for (int n = 0; n < count; n++)
      {
        if (facet_num_a1 == -1)
        {
          facet_num_a1 = many_shared_facet_ids[n];
        }
        for (int i = 0; i < 3; i++)
        {
          if (many_shared_facet_ids[n] == stl->neighbors_start[facet_num_a1].neighbor[i])
          {
            if (facet_num_a2 == -1)
            {
              facet_num_a2 = stl->neighbors_start[facet_num_a1].neighbor[i];
            }
          }
        }
      }
      for (int n = 0; n < count; n++)
      {
        if ((many_shared_facet_ids[n] != facet_num_a1) && (many_shared_facet_ids[n] != facet_num_a2))
        {
          if (facet_num_b1 == -1)
          {
            facet_num_b1 = many_shared_facet_ids[n];
          }
        }
      }
      for (int n = 0; n < count; n++)
      {
        if ((many_shared_facet_ids[n] != facet_num_a1) && (many_shared_facet_ids[n] != facet_num_a2) && (many_shared_facet_ids[n] != facet_num_b1))
        {
          if (facet_num_b2 == -1)
          {
            facet_num_b2 = many_shared_facet_ids[n];
          }
        }
      }
      // 邻居匹配失败
      if (facet_num_a1 == -1 || facet_num_a2 == -1 || facet_num_b1 == -1 || facet_num_b2 == -1)
      {
        is_neighbor_match_well = false;
      }
      if (auto_repair && is_neighbor_match_well)
      {
        is_neighbor_match_well = stl_swap_facets_neighbor(stl, facet_num_a1, facet_num_a2, facet_num_b1, facet_num_b2);
        if (is_neighbor_match_well == false)
        {
          all_many_shared_facet_ids[removed_count++] = facet_num_a1;
          all_many_shared_facet_ids[removed_count++] = facet_num_a2;
          all_many_shared_facet_ids[removed_count++] = facet_num_b1;
          all_many_shared_facet_ids[removed_count++] = facet_num_b2;
        }
      }
      else
      {
      }
    }
    else if (count > 4)
    {
      more_two_facets_shared_one_edge_count++;
      if (auto_repair)
      {
        // 拿到一组共边面片
      }
      else
      {
        // 拿到一组共边面片
      }
      for (int n = 0; n < count; n++)
      {
        all_many_shared_facet_ids[removed_count++] = many_shared_facet_ids[n];
      }
    }
    else
    {
      if (count != 2)
      {
        for (int n = 0; n < count; n++)
        {
          int facet_num = many_shared_facet_ids[n];
          for (int j = 0; j < 3; j++)
          {
            if (stl->neighbors_start[facet_num].which_vertex_not[j] > 2)
            {
              if (auto_repair)
              {
                stl->neighbors_start[facet_num].is_invalid[j] = true;
              }
            }
          }
        }
        stl->stats.no_need_fix_normal_direction = true;
      }
    }
  }

  if (normal_error_count != 0)
  {
    stl->repair_stats.non_manifold = more_two_facets_shared_one_edge_count;
    if (normal_error_count == more_two_facets_shared_one_edge_count && auto_repair && is_neighbor_match_well) // 不能存在邻居交换失败的情况
    {
      stl->stats.no_need_fix_normal_direction = true;
    }
    if (stl->stats.no_need_fix_normal_direction == false)
    {
      stl->repair_stats.inverted_normals = normal_error_count;
    }
  }
  if ((removed_count != 0) && (stl->stats.no_need_fix_normal_direction == false) && auto_repair)
  {
    for (int i = 0; i < stl->stats.number_of_facets; i++)
    {
      has_deleted_facet_ids[i] = -1;
    }

    // 一次性全部删除
    for (int i = 0; i < removed_count; i++)
    {
      if (has_deleted_facet_ids[all_many_shared_facet_ids[i]] == 1)
      {
        continue;
      }
      stl_remove_degenerate(stl, all_many_shared_facet_ids[i]);
      has_deleted_facet_ids[all_many_shared_facet_ids[i]] = 1;
    }
    stl_repair(stl, 0, 0, 1, 1, 1, 1, 1, 2, 0, 0, 0, 0, 0, 0);
  }

  free(all_many_shared_facet_ids);
  free(checked_facet_ids);
  free(normal_error_loop);
  free(many_shared_facet_ids);
  free(has_deleted_facet_ids);
}

static bool stl_swap_facets_neighbor(stl_file *stl, int facet_num_a1, int facet_num_a2, int facet_num_b1, int facet_num_b2)
{
  float tolerance = 0.000001; // 过滤掉面积为 0 的异常面片
  if (stl_is_degenerate_tiny_facet(stl, facet_num_a1, tolerance) == true)
  {
    return false;
  }
  if (stl_is_degenerate_tiny_facet(stl, facet_num_a2, tolerance) == true)
  {
    return false;
  }
  if (stl_is_degenerate_tiny_facet(stl, facet_num_b1, tolerance) == true)
  {
    return false;
  }
  if (stl_is_degenerate_tiny_facet(stl, facet_num_b2, tolerance) == true)
  {
    return false;
  }

  int facet_num_a1_v_not = -1;
  int facet_num_a2_v_not = -1;
  int facet_num_b1_v_not = -1;
  int facet_num_b2_v_not = -1;

  for (int i = 0; i < 3; i++)
  {
    if (stl->neighbors_start[facet_num_a1].which_vertex_not[i] > 2)
    {
      facet_num_a2_v_not = stl->neighbors_start[facet_num_a1].which_vertex_not[i];
    }
    if (stl->neighbors_start[facet_num_a2].which_vertex_not[i] > 2)
    {
      facet_num_a1_v_not = stl->neighbors_start[facet_num_a2].which_vertex_not[i];
    }
    if (stl->neighbors_start[facet_num_b1].which_vertex_not[i] > 2)
    {
      facet_num_b2_v_not = stl->neighbors_start[facet_num_b1].which_vertex_not[i];
    }
    if (stl->neighbors_start[facet_num_b2].which_vertex_not[i] > 2)
    {
      facet_num_b1_v_not = stl->neighbors_start[facet_num_b2].which_vertex_not[i];
    }
  }

  int time = 0;

  // record neib
  int facet_num_a1_n = -1;
  int facet_num_a1_w = -1;
  int facet_num_a1_i = -1;
  int facet_num_a2_n = -1;
  int facet_num_a2_w = -1;
  int facet_num_a2_i = -1;
  int facet_num_b1_n = -1;
  int facet_num_b1_w = -1;
  int facet_num_b1_i = -1;
  int facet_num_b2_n = -1;
  int facet_num_b2_w = -1;
  int facet_num_b2_i = -1;

  for (int i = 0; i < 3; i++)
  {
    if (stl->neighbors_start[facet_num_a1].which_vertex_not[i] > 2)
    {
      if (facet_num_a1_n == -1 && facet_num_a1_w == -1 && facet_num_a1_i == -1)
      {
        facet_num_a1_n = stl->neighbors_start[facet_num_a1].neighbor[i];
        facet_num_a1_w = stl->neighbors_start[facet_num_a1].which_vertex_not[i];
        facet_num_a1_i = i;
        stl->neighbors_start[facet_num_a1].neighbor[i] = facet_num_b2;
        stl->neighbors_start[facet_num_a1].which_vertex_not[i] = facet_num_b2_v_not % 3;
      }
      else
      {
        time++;
      }
    }
    if (stl->neighbors_start[facet_num_a2].which_vertex_not[i] > 2)
    {
      if (facet_num_a2_n == -1 && facet_num_a2_w == -1 && facet_num_a2_i == -1)
      {
        facet_num_a2_n = stl->neighbors_start[facet_num_a2].neighbor[i];
        facet_num_a2_w = stl->neighbors_start[facet_num_a2].which_vertex_not[i];
        facet_num_a2_i = i;
        stl->neighbors_start[facet_num_a2].neighbor[i] = facet_num_b1;
        stl->neighbors_start[facet_num_a2].which_vertex_not[i] = facet_num_b1_v_not % 3;
      }
      else
      {
        time++;
      }
    }
    if (stl->neighbors_start[facet_num_b1].which_vertex_not[i] > 2)
    {
      if (facet_num_b1_n == -1 && facet_num_b1_w == -1 && facet_num_b1_i == -1)
      {
        facet_num_b1_n = stl->neighbors_start[facet_num_b1].neighbor[i];
        facet_num_b1_w = stl->neighbors_start[facet_num_b1].which_vertex_not[i];
        facet_num_b1_i = i;
        stl->neighbors_start[facet_num_b1].neighbor[i] = facet_num_a2;
        stl->neighbors_start[facet_num_b1].which_vertex_not[i] = facet_num_a2_v_not % 3;
      }
      else
      {
        time++;
      }
    }
    if (stl->neighbors_start[facet_num_b2].which_vertex_not[i] > 2)
    {
      if (facet_num_b2_n == -1 && facet_num_b2_w == -1 && facet_num_b2_i == -1)
      {
        facet_num_b2_n = stl->neighbors_start[facet_num_b2].neighbor[i];
        facet_num_b2_w = stl->neighbors_start[facet_num_b2].which_vertex_not[i];
        facet_num_b2_i = i;
        stl->neighbors_start[facet_num_b2].neighbor[i] = facet_num_a1;
        stl->neighbors_start[facet_num_b2].which_vertex_not[i] = facet_num_a1_v_not % 3;
      }
      else
      {
        time++;
      }
    }
  }
  if (time > 0)
  {
    stl->neighbors_start[facet_num_a1].neighbor[facet_num_a1_i] = facet_num_a1_n;
    stl->neighbors_start[facet_num_a1].which_vertex_not[facet_num_a1_i] = facet_num_a1_w;
    stl->neighbors_start[facet_num_a2].neighbor[facet_num_a2_i] = facet_num_a2_n;
    stl->neighbors_start[facet_num_a2].which_vertex_not[facet_num_a2_i] = facet_num_a2_w;
    stl->neighbors_start[facet_num_b1].neighbor[facet_num_b1_i] = facet_num_b1_n;
    stl->neighbors_start[facet_num_b1].which_vertex_not[facet_num_b1_i] = facet_num_b1_w;
    stl->neighbors_start[facet_num_b2].neighbor[facet_num_b2_i] = facet_num_b2_n;
    stl->neighbors_start[facet_num_b2].which_vertex_not[facet_num_b2_i] = facet_num_b2_w;
    return false;
  }
  return true;
}

void stl_repair_init(stl_file *stl)
{
  stl->repair_stats.inverted_normals = 0;
  stl->repair_stats.bad_edges = 0;
  stl->repair_stats.bad_contours = 0;
  stl->repair_stats.near_bad_edges = 0;
  stl->repair_stats.planar_holes = 0;
  stl->repair_stats.noise_shells = 0;
  stl->repair_stats.overlapping_triangles = 0;
  stl->repair_stats.intersecting_triangles = 0;
  stl->repair_stats.non_manifold = 0;
}

void stl_check_facets_exact(stl_file *stl)
{
  /* This function builds the neighbors list.  No modifications are made
   *  to any of the facets.  The edges are said to match only if all six
   *  floats of the first edge matches all six floats of the second edge.
   */
  /*
  此方法创建相邻关系列表。不会对面片本身信息造成任何修改。
  判断两个面片相邻的条件仅为当两条边的端点三维坐标完全重合时。
  */
  stl_hash_edge edge;
  stl_facet facet;
  int i;
  int j;

  if (stl->error)
    return;

  stl->stats.connected_edges = 0;
  stl->stats.connected_facets_1_edge = 0;
  stl->stats.connected_facets_2_edge = 0;
  stl->stats.connected_facets_3_edge = 0;

  stl_initialize_facet_check_exact(stl);

  for (i = 0; i < stl->stats.number_of_facets; i++)
  {
    facet = stl->facet_start[i];
    // Positive and negative zeros are possible in the floats, which are considered equal by the FP unit.
    // When using a memcmp on raw floats, those numbers report to be different.
    // Unify all +0 and -0 to +0 to make the floats equal under memcmp.
    {
      uint32_t *f = (uint32_t *)&facet;
      for (int j = 0; j < 12; ++j, ++f) // 3x vertex + normal: 4x3 = 12 floats
        if (*f == 0x80000000)
          // Negative zero, switch to positive zero.
          *f = 0;
    }

    /* If any two of the three vertices are found to be exactally the same, call them degenerate and remove the facet. */
    if (!memcmp(&facet.vertex[0], &facet.vertex[1],
                sizeof(stl_vertex)) ||
        !memcmp(&facet.vertex[1], &facet.vertex[2],
                sizeof(stl_vertex)) ||
        !memcmp(&facet.vertex[0], &facet.vertex[2],
                sizeof(stl_vertex)))
    {
      stl->stats.degenerate_facets += 1;
      stl_remove_facet(stl, i);
      i--;
      continue;
    }
    /*
    for (int k = i + 1; k < stl->stats.number_of_facets; k++)
    {
      stl_facet temp_facet = stl->facet_start[k];
      if (!memcmp(&facet.vertex[0], &temp_facet.vertex[0], sizeof(stl_vertex))
        || !memcmp(&facet.vertex[0], &temp_facet.vertex[1], sizeof(stl_vertex))
        || !memcmp(&facet.vertex[0], &temp_facet.vertex[2], sizeof(stl_vertex)))
      {
        if (!memcmp(&facet.vertex[1], &temp_facet.vertex[0], sizeof(stl_vertex))
          || !memcmp(&facet.vertex[1], &temp_facet.vertex[1], sizeof(stl_vertex))
          || !memcmp(&facet.vertex[1], &temp_facet.vertex[2], sizeof(stl_vertex)))
        {
          if (!memcmp(&facet.vertex[2], &temp_facet.vertex[0], sizeof(stl_vertex))
            || !memcmp(&facet.vertex[2], &temp_facet.vertex[1], sizeof(stl_vertex))
            || !memcmp(&facet.vertex[2], &temp_facet.vertex[2], sizeof(stl_vertex)))
          {
            stl->stats.degenerate_facets += 1;
            stl_remove_facet(stl, k);
            k--;
          }
        }
      }
    }
    */
    /*
    for (int k = i + 1; k < stl->stats.number_of_facets; k++)
    {
      stl_facet temp_facet = stl->facet_start[k];
      float facet_area = get_area(&facet);
      float temp_facet_area = get_area(&temp_facet);
      if ((fabs(facet_area - temp_facet_area) < 1e-6) && stl_is_two_facet_almost_coplane(facet,temp_facet,1e-6))
      {
        stl->stats.degenerate_facets += 1;
        stl_remove_facet(stl, k);
        k--;
      }
    }
    */

    for (j = 0; j < 3; j++)
    {
      edge.facet_number = i;
      edge.which_edge = j;
      stl_load_edge_exact(stl, &edge, &facet.vertex[j],
                          &facet.vertex[(j + 1) % 3]);

      insert_hash_edge(stl, edge, stl_match_neighbors_exact);
    }
  }
  stl_free_edges(stl);

#if 0
  printf("Number of faces: %d, number of manifold edges: %d, number of connected edges: %d, number of unconnected edges: %d\r\n", 
    stl->stats.number_of_facets, stl->stats.number_of_facets * 3, 
    stl->stats.connected_edges, stl->stats.number_of_facets * 3 - stl->stats.connected_edges);
#endif
}

static void
stl_load_edge_exact(stl_file *stl, stl_hash_edge *edge,
                    stl_vertex *a, stl_vertex *b)
{

  if (stl->error)
    return;

  {
    float diff_x = ABS(a->x - b->x);
    float diff_y = ABS(a->y - b->y);
    float diff_z = ABS(a->z - b->z);
    float max_diff = STL_MAX(diff_x, diff_y);
    max_diff = STL_MAX(diff_z, max_diff);
    stl->stats.shortest_edge = STL_MIN(max_diff, stl->stats.shortest_edge);
  }

  // Ensure identical vertex ordering of equal edges.
  // This method is numerically robust.
  if ((a->x != b->x) ? (a->x < b->x) : ((a->y != b->y) ? (a->y < b->y) : (a->z < b->z)))
  {
    memcpy(&edge->key[0], a, sizeof(stl_vertex));
    memcpy(&edge->key[3], b, sizeof(stl_vertex));
  }
  else
  {
    memcpy(&edge->key[0], b, sizeof(stl_vertex));
    memcpy(&edge->key[3], a, sizeof(stl_vertex));
    edge->which_edge += 3; /* this edge is loaded backwards */
  }
}

static void
stl_initialize_facet_check_exact(stl_file *stl)
{
  int i;

  if (stl->error)
    return;

  stl->stats.malloced = 0;
  stl->stats.freed = 0;
  stl->stats.collisions = 0;

  stl->M = 81397;

  for (i = 0; i < stl->stats.number_of_facets; i++)
  {
    /* initialize neighbors list to -1 to mark unconnected edges */
    stl->neighbors_start[i].neighbor[0] = -1;
    stl->neighbors_start[i].neighbor[1] = -1;
    stl->neighbors_start[i].neighbor[2] = -1;

    stl->neighbors_start[i].is_invalid[0] = false;
    stl->neighbors_start[i].is_invalid[1] = false;
    stl->neighbors_start[i].is_invalid[2] = false;
  }

  stl->heads = (stl_hash_edge **)calloc(stl->M, sizeof(*stl->heads));
  if (stl->heads == NULL)
    perror("stl_initialize_facet_check_exact");

  stl->tail = (stl_hash_edge *)malloc(sizeof(stl_hash_edge));
  if (stl->tail == NULL)
    perror("stl_initialize_facet_check_exact");

  stl->tail->next = stl->tail;

  for (i = 0; i < stl->M; i++)
  {
    stl->heads[i] = stl->tail;
  }
}

static void
insert_hash_edge(stl_file *stl, stl_hash_edge edge,
                 void (*match_neighbors)(stl_file *stl,
                                         stl_hash_edge *edge_a, stl_hash_edge *edge_b))
{
  stl_hash_edge *link;
  stl_hash_edge *new_edge;
  stl_hash_edge *temp;
  int chain_number;

  if (stl->error)
    return;

  chain_number = stl_get_hash_for_edge(stl->M, &edge);

  link = stl->heads[chain_number];

  if (link == stl->tail)
  {
    /* This list doesn't have any edges currently in it.  Add this one. */
    new_edge = (stl_hash_edge *)malloc(sizeof(stl_hash_edge));
    if (new_edge == NULL)
      perror("insert_hash_edge");
    stl->stats.malloced++;
    *new_edge = edge;
    new_edge->next = stl->tail;
    stl->heads[chain_number] = new_edge;
    return;
  }
  else if (!stl_compare_function(&edge, link))
  {
    /* This is a match.  Record result in neighbors list. */
    match_neighbors(stl, &edge, link);
    /* Delete the matched edge from the list. */
    stl->heads[chain_number] = link->next;
    free(link);
    stl->stats.freed++;
    return;
  }
  else
  {
    /* Continue through the rest of the list */
    for (;;)
    {
      if (link->next == stl->tail)
      {
        /* This is the last item in the list. Insert a new edge. */
        new_edge = (stl_hash_edge *)malloc(sizeof(stl_hash_edge));
        if (new_edge == NULL)
          perror("insert_hash_edge");
        stl->stats.malloced++;
        *new_edge = edge;
        new_edge->next = stl->tail;
        link->next = new_edge;
        stl->stats.collisions++;
        return;
      }
      else if (!stl_compare_function(&edge, link->next))
      {
        /* This is a match.  Record result in neighbors list. */
        match_neighbors(stl, &edge, link->next);

        /* Delete the matched edge from the list. */
        temp = link->next;
        link->next = link->next->next;
        free(temp);
        stl->stats.freed++;
        return;
      }
      else
      {
        /* This is not a match.  Go to the next link */
        link = link->next;
        stl->stats.collisions++;
      }
    }
  }
}

static int
stl_get_hash_for_edge(int M, stl_hash_edge *edge)
{
  return ((edge->key[0] / 23 + edge->key[1] / 19 + edge->key[2] / 17 + edge->key[3] / 13 + edge->key[4] / 11 + edge->key[5] / 7) % M);
}

static int
stl_compare_function(stl_hash_edge *edge_a, stl_hash_edge *edge_b)
{
  if (edge_a->facet_number == edge_b->facet_number)
  {
    return 1; /* Don't match edges of the same facet */
  }
  else
  {
    return memcmp(edge_a, edge_b, SIZEOF_EDGE_SORT);
  }
}

void stl_check_facets_nearby(stl_file *stl, float tolerance)
{
  stl_hash_edge edge[3];
  stl_facet facet;
  int i;
  int j;

  if (stl->error)
    return;

  if ((stl->stats.connected_facets_1_edge == stl->stats.number_of_facets) && (stl->stats.connected_facets_2_edge == stl->stats.number_of_facets) && (stl->stats.connected_facets_3_edge == stl->stats.number_of_facets))
  {
    /* No need to check any further.  All facets are connected */
    return;
  }

  stl_initialize_facet_check_nearby(stl);

  for (i = 0; i < stl->stats.number_of_facets; i++)
  {
    facet = stl->facet_start[i];
    // Positive and negative zeros are possible in the floats, which are considered equal by the FP unit.
    // When using a memcmp on raw floats, those numbers report to be different.
    // Unify all +0 and -0 to +0 to make the floats equal under memcmp.
    {
      uint32_t *f = (uint32_t *)&facet;
      for (int j = 0; j < 12; ++j, ++f) // 3x vertex + normal: 4x3 = 12 floats
        if (*f == 0x80000000)
          // Negative zero, switch to positive zero.
          *f = 0;
    }
    for (j = 0; j < 3; j++)
    {
      if (stl->neighbors_start[i].neighbor[j] == -1)
      {
        edge[j].facet_number = i;
        edge[j].which_edge = j;
        if (stl_load_edge_nearby(stl, &edge[j], &facet.vertex[j],
                                 &facet.vertex[(j + 1) % 3],
                                 tolerance))
        {
          /* only insert edges that have different keys */
          insert_hash_edge(stl, edge[j], stl_match_neighbors_nearby);
        }
      }
    }
  }

  stl_free_edges(stl);
}

static int
stl_load_edge_nearby(stl_file *stl, stl_hash_edge *edge,
                     stl_vertex *a, stl_vertex *b, float tolerance)
{
  // Index of a grid cell spaced by tolerance.
  uint32_t vertex1[3] = {
      (uint32_t)((a->x - stl->stats.min.x) / tolerance),
      (uint32_t)((a->y - stl->stats.min.y) / tolerance),
      (uint32_t)((a->z - stl->stats.min.z) / tolerance)};
  uint32_t vertex2[3] = {
      (uint32_t)((b->x - stl->stats.min.x) / tolerance),
      (uint32_t)((b->y - stl->stats.min.y) / tolerance),
      (uint32_t)((b->z - stl->stats.min.z) / tolerance)};

  if ((vertex1[0] == vertex2[0]) && (vertex1[1] == vertex2[1]) && (vertex1[2] == vertex2[2]))
  {
    /* Both vertices hash to the same value */
    return 0;
  }

  // Ensure identical vertex ordering of edges, which vertices land into equal grid cells.
  // This method is numerically robust.
  if ((vertex1[0] != vertex2[0]) ? (vertex1[0] < vertex2[0]) : ((vertex1[1] != vertex2[1]) ? (vertex1[1] < vertex2[1]) : (vertex1[2] < vertex2[2])))
  {
    memcpy(&edge->key[0], vertex1, sizeof(stl_vertex));
    memcpy(&edge->key[3], vertex2, sizeof(stl_vertex));
  }
  else
  {
    memcpy(&edge->key[0], vertex2, sizeof(stl_vertex));
    memcpy(&edge->key[3], vertex1, sizeof(stl_vertex));
    edge->which_edge += 3; /* this edge is loaded backwards */
  }
  return 1;
}

static void
stl_free_edges(stl_file *stl)
{
  int i;
  stl_hash_edge *temp;

  if (stl->error)
    return;

  if (stl->stats.malloced != stl->stats.freed)
  {
    for (i = 0; i < stl->M; i++)
    {
      for (temp = stl->heads[i]; stl->heads[i] != stl->tail;
           temp = stl->heads[i])
      {
        stl->heads[i] = stl->heads[i]->next;
        free(temp);
        stl->stats.freed++;
      }
    }
  }
  free(stl->heads);
  free(stl->tail);
}

static void
stl_initialize_facet_check_nearby(stl_file *stl)
{
  int i;

  if (stl->error)
    return;

  stl->stats.malloced = 0;
  stl->stats.freed = 0;
  stl->stats.collisions = 0;

  /*  tolerance = STL_MAX(stl->stats.shortest_edge, tolerance);*/
  /*  tolerance = STL_MAX((stl->stats.bounding_diameter / 500000.0), tolerance);*/
  /*  tolerance *= 0.5;*/

  stl->M = 81397;

  stl->heads = (stl_hash_edge **)calloc(stl->M, sizeof(*stl->heads));
  if (stl->heads == NULL)
    perror("stl_initialize_facet_check_nearby");

  stl->tail = (stl_hash_edge *)malloc(sizeof(stl_hash_edge));
  if (stl->tail == NULL)
    perror("stl_initialize_facet_check_nearby");

  stl->tail->next = stl->tail;

  for (i = 0; i < stl->M; i++)
  {
    stl->heads[i] = stl->tail;
  }
}

static void
stl_record_neighbors(stl_file *stl,
                     stl_hash_edge *edge_a, stl_hash_edge *edge_b)
{
  int i;
  int j;

  if (stl->error)
    return;

  /* Facet a's neighbor is facet b */
  stl->neighbors_start[edge_a->facet_number].neighbor[edge_a->which_edge % 3] =
      edge_b->facet_number; /* sets the .neighbor part */

  stl->neighbors_start[edge_a->facet_number].which_vertex_not[edge_a->which_edge % 3] =
      (edge_b->which_edge + 2) % 3; /* sets the .which_vertex_not part */

  /* Facet b's neighbor is facet a */
  stl->neighbors_start[edge_b->facet_number].neighbor[edge_b->which_edge % 3] =
      edge_a->facet_number; /* sets the .neighbor part */

  stl->neighbors_start[edge_b->facet_number].which_vertex_not[edge_b->which_edge % 3] =
      (edge_a->which_edge + 2) % 3; /* sets the .which_vertex_not part */

  if (((edge_a->which_edge < 3) && (edge_b->which_edge < 3)) || ((edge_a->which_edge > 2) && (edge_b->which_edge > 2)))
  {
    /* these facets are oriented in opposite directions.  */
    /*  their normals are probably messed up. */
    stl->neighbors_start[edge_a->facet_number].which_vertex_not[edge_a->which_edge % 3] += 3;
    stl->neighbors_start[edge_b->facet_number].which_vertex_not[edge_b->which_edge % 3] += 3;
  }

  /* Count successful connects */
  /* Total connects */
  stl->stats.connected_edges += 2;
  /* Count individual connects */
  i = ((stl->neighbors_start[edge_a->facet_number].neighbor[0] == -1) +
       (stl->neighbors_start[edge_a->facet_number].neighbor[1] == -1) +
       (stl->neighbors_start[edge_a->facet_number].neighbor[2] == -1));
  j = ((stl->neighbors_start[edge_b->facet_number].neighbor[0] == -1) +
       (stl->neighbors_start[edge_b->facet_number].neighbor[1] == -1) +
       (stl->neighbors_start[edge_b->facet_number].neighbor[2] == -1));
  if (i == 2)
  {
    stl->stats.connected_facets_1_edge += 1;
  }
  else if (i == 1)
  {
    stl->stats.connected_facets_2_edge += 1;
  }
  else
  {
    stl->stats.connected_facets_3_edge += 1;
  }
  if (j == 2)
  {
    stl->stats.connected_facets_1_edge += 1;
  }
  else if (j == 1)
  {
    stl->stats.connected_facets_2_edge += 1;
  }
  else
  {
    stl->stats.connected_facets_3_edge += 1;
  }
}

static void
stl_match_neighbors_exact(stl_file *stl,
                          stl_hash_edge *edge_a, stl_hash_edge *edge_b)
{
  if (stl->error)
    return;
  stl_record_neighbors(stl, edge_a, edge_b);
}

static void
stl_match_neighbors_nearby(stl_file *stl,
                           stl_hash_edge *edge_a, stl_hash_edge *edge_b)
{
  int facet1;
  int facet2;
  int vertex1;
  int vertex2;
  int vnot1;
  int vnot2;
  stl_vertex new_vertex1;
  stl_vertex new_vertex2;

  if (stl->error)
    return;

  stl_record_neighbors(stl, edge_a, edge_b);
  stl_which_vertices_to_change(stl, edge_a, edge_b, &facet1, &vertex1,
                               &facet2, &vertex2, &new_vertex1, &new_vertex2);
  if (facet1 != -1)
  {
    if (facet1 == edge_a->facet_number)
    {
      vnot1 = (edge_a->which_edge + 2) % 3;
    }
    else
    {
      vnot1 = (edge_b->which_edge + 2) % 3;
    }
    if (((vnot1 + 2) % 3) == vertex1)
    {
      vnot1 += 3;
    }
    stl_change_vertices(stl, facet1, vnot1, new_vertex1);
  }
  if (facet2 != -1)
  {
    if (facet2 == edge_a->facet_number)
    {
      vnot2 = (edge_a->which_edge + 2) % 3;
    }
    else
    {
      vnot2 = (edge_b->which_edge + 2) % 3;
    }
    if (((vnot2 + 2) % 3) == vertex2)
    {
      vnot2 += 3;
    }
    stl_change_vertices(stl, facet2, vnot2, new_vertex2);
  }
  stl->stats.edges_fixed += 2;
}

static void
stl_change_vertices(stl_file *stl, int facet_num, int vnot,
                    stl_vertex new_vertex)
{
  int first_facet;
  int direction;
  int next_edge;
  int pivot_vertex;

  if (stl->error)
    return;

  first_facet = facet_num;
  direction = 0;

  for (;;)
  {
    if (vnot > 2)
    {
      if (direction == 0)
      {
        pivot_vertex = (vnot + 2) % 3;
        next_edge = pivot_vertex;
        direction = 1;
      }
      else
      {
        pivot_vertex = (vnot + 1) % 3;
        next_edge = vnot % 3;
        direction = 0;
      }
    }
    else
    {
      if (direction == 0)
      {
        pivot_vertex = (vnot + 1) % 3;
        next_edge = vnot;
      }
      else
      {
        pivot_vertex = (vnot + 2) % 3;
        next_edge = pivot_vertex;
      }
    }
#if 0
    if (stl->facet_start[facet_num].vertex[pivot_vertex].x == new_vertex.x &&
        stl->facet_start[facet_num].vertex[pivot_vertex].y == new_vertex.y &&
        stl->facet_start[facet_num].vertex[pivot_vertex].z == new_vertex.z)
      printf("Changing vertex %f,%f,%f: Same !!!\r\n", 
        new_vertex.x, new_vertex.y, new_vertex.z);
    else {
      if (stl->facet_start[facet_num].vertex[pivot_vertex].x != new_vertex.x)
        printf("Changing coordinate x, vertex %e (0x%08x) to %e(0x%08x)\r\n", 
          stl->facet_start[facet_num].vertex[pivot_vertex].x,
          *reinterpret_cast<const int*>(&stl->facet_start[facet_num].vertex[pivot_vertex].x),
          new_vertex.x,
          *reinterpret_cast<const int*>(&new_vertex.x));
      if (stl->facet_start[facet_num].vertex[pivot_vertex].y != new_vertex.y)
        printf("Changing coordinate x, vertex %e (0x%08x) to %e(0x%08x)\r\n", 
          stl->facet_start[facet_num].vertex[pivot_vertex].y,
          *reinterpret_cast<const int*>(&stl->facet_start[facet_num].vertex[pivot_vertex].y),
          new_vertex.y,
          *reinterpret_cast<const int*>(&new_vertex.y));
      if (stl->facet_start[facet_num].vertex[pivot_vertex].z != new_vertex.z)
        printf("Changing coordinate x, vertex %e (0x%08x) to %e(0x%08x)\r\n", 
          stl->facet_start[facet_num].vertex[pivot_vertex].z,
          *reinterpret_cast<const int*>(&stl->facet_start[facet_num].vertex[pivot_vertex].z),
          new_vertex.z,
          *reinterpret_cast<const int*>(&new_vertex.z));
    }
#endif
    stl->facet_start[facet_num].vertex[pivot_vertex] = new_vertex;
    vnot = stl->neighbors_start[facet_num].which_vertex_not[next_edge];
    facet_num = stl->neighbors_start[facet_num].neighbor[next_edge];

    if (facet_num == -1)
    {
      break;
    }

    if (facet_num == first_facet)
    {
      /* back to the beginning */
      LOGINFO("Back to the first facet changing vertices: probably a mobius part.\n. Try using a smaller tolerance or don't do a nearby check\n");
      return;
    }
  }
}

static void
stl_which_vertices_to_change(stl_file *stl, stl_hash_edge *edge_a,
                             stl_hash_edge *edge_b, int *facet1, int *vertex1,
                             int *facet2, int *vertex2,
                             stl_vertex *new_vertex1, stl_vertex *new_vertex2)
{
  int v1a; /* pair 1, facet a */
  int v1b; /* pair 1, facet b */
  int v2a; /* pair 2, facet a */
  int v2b; /* pair 2, facet b */

  /* Find first pair */
  if (edge_a->which_edge < 3)
  {
    v1a = edge_a->which_edge;
    v2a = (edge_a->which_edge + 1) % 3;
  }
  else
  {
    v2a = edge_a->which_edge % 3;
    v1a = (edge_a->which_edge + 1) % 3;
  }
  if (edge_b->which_edge < 3)
  {
    v1b = edge_b->which_edge;
    v2b = (edge_b->which_edge + 1) % 3;
  }
  else
  {
    v2b = edge_b->which_edge % 3;
    v1b = (edge_b->which_edge + 1) % 3;
  }

  /* Of the first pair, which vertex, if any, should be changed */
  if (!memcmp(&stl->facet_start[edge_a->facet_number].vertex[v1a],
              &stl->facet_start[edge_b->facet_number].vertex[v1b],
              sizeof(stl_vertex)))
  {
    /* These facets are already equal.  No need to change. */
    *facet1 = -1;
  }
  else
  {
    if ((stl->neighbors_start[edge_a->facet_number].neighbor[v1a] == -1) && (stl->neighbors_start[edge_a->facet_number].neighbor[(v1a + 2) % 3] == -1))
    {
      /* This vertex has no neighbors.  This is a good one to change */
      *facet1 = edge_a->facet_number;
      *vertex1 = v1a;
      *new_vertex1 = stl->facet_start[edge_b->facet_number].vertex[v1b];
    }
    else
    {
      *facet1 = edge_b->facet_number;
      *vertex1 = v1b;
      *new_vertex1 = stl->facet_start[edge_a->facet_number].vertex[v1a];
    }
  }

  /* Of the second pair, which vertex, if any, should be changed */
  if (!memcmp(&stl->facet_start[edge_a->facet_number].vertex[v2a],
              &stl->facet_start[edge_b->facet_number].vertex[v2b],
              sizeof(stl_vertex)))
  {
    /* These facets are already equal.  No need to change. */
    *facet2 = -1;
  }
  else
  {
    if ((stl->neighbors_start[edge_a->facet_number].neighbor[v2a] == -1) && (stl->neighbors_start[edge_a->facet_number].neighbor[(v2a + 2) % 3] == -1))
    {
      /* This vertex has no neighbors.  This is a good one to change */
      *facet2 = edge_a->facet_number;
      *vertex2 = v2a;
      *new_vertex2 = stl->facet_start[edge_b->facet_number].vertex[v2b];
    }
    else
    {
      *facet2 = edge_b->facet_number;
      *vertex2 = v2b;
      *new_vertex2 = stl->facet_start[edge_a->facet_number].vertex[v2a];
    }
  }
}

void stl_remove_facet(stl_file *stl, int facet_number)
{
  int neighbor[3];
  int vnot[3];
  int i;
  int j;

  if (stl->error)
    return;

  stl->stats.facets_removed += 1;
  /* Update list of connected edges */
  j = ((stl->neighbors_start[facet_number].neighbor[0] == -1) +
       (stl->neighbors_start[facet_number].neighbor[1] == -1) +
       (stl->neighbors_start[facet_number].neighbor[2] == -1));
  if (j == 2)
  {
    stl->stats.connected_facets_1_edge -= 1;
  }
  else if (j == 1)
  {
    stl->stats.connected_facets_2_edge -= 1;
    stl->stats.connected_facets_1_edge -= 1;
  }
  else if (j == 0)
  {
    stl->stats.connected_facets_3_edge -= 1;
    stl->stats.connected_facets_2_edge -= 1;
    stl->stats.connected_facets_1_edge -= 1;
  }

  stl->facet_start[facet_number] =
      stl->facet_start[stl->stats.number_of_facets - 1];
  /* I could reallocate at this point, but it is not really necessary. */
  stl->neighbors_start[facet_number] =
      stl->neighbors_start[stl->stats.number_of_facets - 1];
  stl->stats.number_of_facets -= 1;

  for (i = 0; i < 3; i++)
  {
    neighbor[i] = stl->neighbors_start[facet_number].neighbor[i];
    vnot[i] = stl->neighbors_start[facet_number].which_vertex_not[i];
  }
  for (i = 0; i < 3; i++)
  {
    if (neighbor[i] != -1)
    {
      if (stl->neighbors_start[neighbor[i]].neighbor[(vnot[i] + 1) % 3] !=
          stl->stats.number_of_facets)
      {
        stl->stats.unable_repair = true;
        LOGINFO("in stl_remove_facet: neighbor = %d numfacets = %d this  is wrong \n",
                stl->neighbors_start[neighbor[i]].neighbor[(vnot[i] + 1) % 3],
                stl->stats.number_of_facets);
        return;
      }
      stl->neighbors_start[neighbor[i]].neighbor[(vnot[i] + 1) % 3] = facet_number;
    }
  }
}

void stl_remove_unconnected_facets(stl_file *stl)
{
  /* A couple of things need to be done here.  One is to remove any */
  /* completely unconnected facets (0 edges connected) since these are */
  /* useless and could be completely wrong.   The second thing that needs to */
  /* be done is to remove any degenerate facets that were created during */
  /* stl_check_facets_nearby(). */

  int i;

  LOGINFO("stl->error %d || stl->stats.unable_repair %d", stl->error, stl->stats.unable_repair);
  if (stl->error || stl->stats.unable_repair)
    return;

  /* remove degenerate facets */
  for (i = 0; i < stl->stats.number_of_facets; i++)
  {
    if (!memcmp(&stl->facet_start[i].vertex[0],
                &stl->facet_start[i].vertex[1], sizeof(stl_vertex)) ||
        !memcmp(&stl->facet_start[i].vertex[1],
                &stl->facet_start[i].vertex[2], sizeof(stl_vertex)) ||
        !memcmp(&stl->facet_start[i].vertex[0],
                &stl->facet_start[i].vertex[2], sizeof(stl_vertex)))
    {
      stl_remove_degenerate(stl, i);
      i--;
    }
  }

  if (stl->stats.connected_facets_1_edge < stl->stats.number_of_facets)
  {
    /* remove completely unconnected facets */
    for (i = 0; i < stl->stats.number_of_facets; i++)
    {
      if ((stl->neighbors_start[i].neighbor[0] == -1) && (stl->neighbors_start[i].neighbor[1] == -1) && (stl->neighbors_start[i].neighbor[2] == -1))
      {
        /* This facet is completely unconnected.  Remove it. */
        stl_remove_facet(stl, i);
        i--;
      }
    }
  }
}

void stl_remove_degenerate(stl_file *stl, int facet)
{
  int edge1;
  int edge2;
  int edge3;
  int neighbor1;
  int neighbor2;
  int neighbor3;
  int vnot1;
  int vnot2;
  int vnot3;

  if (stl->error)
    return;

  if (!memcmp(&stl->facet_start[facet].vertex[0],
              &stl->facet_start[facet].vertex[1], sizeof(stl_vertex)) &&
      !memcmp(&stl->facet_start[facet].vertex[1],
              &stl->facet_start[facet].vertex[2], sizeof(stl_vertex)))
  {
    /* all 3 vertices are equal.  Just remove the facet.  I don't think*/
    /* this is really possible, but just in case... */
    LOGINFO("removing a facet in stl_remove_degenerate\n");

    stl_remove_facet(stl, facet);
    return;
  }

  if (!memcmp(&stl->facet_start[facet].vertex[0],
              &stl->facet_start[facet].vertex[1], sizeof(stl_vertex)))
  {
    edge1 = 1;
    edge2 = 2;
    edge3 = 0;
  }
  else if (!memcmp(&stl->facet_start[facet].vertex[1],
                   &stl->facet_start[facet].vertex[2], sizeof(stl_vertex)))
  {
    edge1 = 0;
    edge2 = 2;
    edge3 = 1;
  }
  else if (!memcmp(&stl->facet_start[facet].vertex[2],
                   &stl->facet_start[facet].vertex[0], sizeof(stl_vertex)))
  {
    edge1 = 0;
    edge2 = 1;
    edge3 = 2;
  }
  else
  {
    /* No degenerate. Function shouldn't have been called. */
    return;
  }
  neighbor1 = stl->neighbors_start[facet].neighbor[edge1];
  neighbor2 = stl->neighbors_start[facet].neighbor[edge2];

  if (neighbor1 == -1)
  {
    stl_update_connects_remove_1(stl, neighbor2);
  }
  if (neighbor2 == -1)
  {
    stl_update_connects_remove_1(stl, neighbor1);
  }

  neighbor3 = stl->neighbors_start[facet].neighbor[edge3];
  vnot1 = stl->neighbors_start[facet].which_vertex_not[edge1];
  vnot2 = stl->neighbors_start[facet].which_vertex_not[edge2];
  vnot3 = stl->neighbors_start[facet].which_vertex_not[edge3];

  if (neighbor1 >= 0)
  {
    stl->neighbors_start[neighbor1].neighbor[(vnot1 + 1) % 3] = neighbor2;
    stl->neighbors_start[neighbor1].which_vertex_not[(vnot1 + 1) % 3] = vnot2;
  }
  if (neighbor2 >= 0)
  {
    stl->neighbors_start[neighbor2].neighbor[(vnot2 + 1) % 3] = neighbor1;
    stl->neighbors_start[neighbor2].which_vertex_not[(vnot2 + 1) % 3] = vnot1;
  }

  stl_remove_facet(stl, facet);

  if (neighbor3 >= 0)
  {
    stl_update_connects_remove_1(stl, neighbor3);
    stl->neighbors_start[neighbor3].neighbor[(vnot3 + 1) % 3] = -1;
  }
}

void stl_update_connects_remove_1(stl_file *stl, int facet_num)
{
  int j;

  if (stl->error)
    return;
  /* Update list of connected edges */
  j = ((stl->neighbors_start[facet_num].neighbor[0] == -1) +
       (stl->neighbors_start[facet_num].neighbor[1] == -1) +
       (stl->neighbors_start[facet_num].neighbor[2] == -1));
  if (j == 0)
  { /* Facet has 3 neighbors */
    stl->stats.connected_facets_3_edge -= 1;
  }
  else if (j == 1)
  { /* Facet has 2 neighbors */
    stl->stats.connected_facets_2_edge -= 1;
  }
  else if (j == 2)
  { /* Facet has 1 neighbor  */
    stl->stats.connected_facets_1_edge -= 1;
  }
}

void stl_fill_holes(stl_file *stl)
{
  stl_facet facet;
  stl_facet new_facet;
  int neighbors_initial[3];
  stl_hash_edge edge;
  int first_facet;
  int direction;
  int facet_num;
  int vnot;
  int next_edge;
  int pivot_vertex;
  int next_facet;
  int i;
  int j;
  int k;

  LOGINFO("stl->error %d || stl->stats.unable_repair %d", stl->error, stl->stats.unable_repair);
  if (stl->error || stl->stats.unable_repair)
    return;

  /* Insert all unconnected edges into hash list */
  stl_initialize_facet_check_nearby(stl);
  for (i = 0; i < stl->stats.number_of_facets; i++)
  {
    facet = stl->facet_start[i];
    for (j = 0; j < 3; j++)
    {
      if (stl->neighbors_start[i].neighbor[j] != -1)
        continue;
      edge.facet_number = i;
      edge.which_edge = j;
      stl_load_edge_exact(stl, &edge, &facet.vertex[j],
                          &facet.vertex[(j + 1) % 3]);

      insert_hash_edge(stl, edge, stl_match_neighbors_exact);
    }
  }

  for (i = 0; i < stl->stats.number_of_facets; i++)
  {
    facet = stl->facet_start[i];
    neighbors_initial[0] = stl->neighbors_start[i].neighbor[0];
    neighbors_initial[1] = stl->neighbors_start[i].neighbor[1];
    neighbors_initial[2] = stl->neighbors_start[i].neighbor[2];
    first_facet = i;
    for (j = 0; j < 3; j++)
    {
      if (stl->neighbors_start[i].neighbor[j] != -1)
        continue;

      new_facet.vertex[0] = facet.vertex[j];
      new_facet.vertex[1] = facet.vertex[(j + 1) % 3];
      if (neighbors_initial[(j + 2) % 3] == -1)
      {
        direction = 1;
      }
      else
      {
        direction = 0;
      }

      facet_num = i;
      vnot = (j + 2) % 3;

      for (;;)
      {
        if (vnot > 2)
        {
          if (direction == 0)
          {
            pivot_vertex = (vnot + 2) % 3;
            next_edge = pivot_vertex;
            direction = 1;
          }
          else
          {
            pivot_vertex = (vnot + 1) % 3;
            next_edge = vnot % 3;
            direction = 0;
          }
        }
        else
        {
          if (direction == 0)
          {
            pivot_vertex = (vnot + 1) % 3;
            next_edge = vnot;
          }
          else
          {
            pivot_vertex = (vnot + 2) % 3;
            next_edge = pivot_vertex;
          }
        }
        next_facet = stl->neighbors_start[facet_num].neighbor[next_edge];

        if (next_facet == -1)
        {
          new_facet.vertex[2] = stl->facet_start[facet_num].vertex[vnot % 3];
          stl_add_facet(stl, &new_facet);
          for (k = 0; k < 3; k++)
          {
            edge.facet_number = stl->stats.number_of_facets - 1;
            edge.which_edge = k;
            stl_load_edge_exact(stl, &edge, &new_facet.vertex[k],
                                &new_facet.vertex[(k + 1) % 3]);

            insert_hash_edge(stl, edge, stl_match_neighbors_exact);
          }
          break;
        }
        else
        {
          vnot = stl->neighbors_start[facet_num].which_vertex_not[next_edge];
          facet_num = next_facet;
        }

        if (facet_num == first_facet)
        {
          /* back to the beginning */
          LOGINFO("Back to the first facet filling holes: probably a mobius part. \n Try using a smaller tolerance or don't do a nearby check \n");
          return;
        }
      }
    }
  }
}

void stl_add_facet(stl_file *stl, stl_facet *new_facet)
{
  if (stl->error)
    return;

  stl->stats.facets_added += 1;
  if (stl->stats.facets_malloced < stl->stats.number_of_facets + 1)
  {
    stl->facet_start = (stl_facet *)realloc(stl->facet_start,
                                            (sizeof(stl_facet) * (stl->stats.facets_malloced + 256)));
    if (stl->facet_start == NULL)
      perror("stl_add_facet");
    stl->neighbors_start = (stl_neighbors *)realloc(stl->neighbors_start,
                                                    (sizeof(stl_neighbors) * (stl->stats.facets_malloced + 256)));
    if (stl->neighbors_start == NULL)
      perror("stl_add_facet");
    stl->stats.facets_malloced += 256;
  }

  if (new_facet->face_side < 0)
    new_facet->face_side = 0;
  if (new_facet->face_type < 0)
    new_facet->face_type = 0;
  if (new_facet->face_type_id < 0)
    new_facet->face_type_id = 0;

  stl->facet_start[stl->stats.number_of_facets] = *new_facet;

  /* note that the normal vector is not set here, just initialized to 0 */
  stl->facet_start[stl->stats.number_of_facets].normal.x = 0.0;
  stl->facet_start[stl->stats.number_of_facets].normal.y = 0.0;
  stl->facet_start[stl->stats.number_of_facets].normal.z = 0.0;

  stl->neighbors_start[stl->stats.number_of_facets].neighbor[0] = -1;
  stl->neighbors_start[stl->stats.number_of_facets].neighbor[1] = -1;
  stl->neighbors_start[stl->stats.number_of_facets].neighbor[2] = -1;
  stl->stats.number_of_facets += 1;
}
