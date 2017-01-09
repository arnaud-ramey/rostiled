/*!
  \file        utils.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2017/1/6
________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file
 */

#ifndef UTILS_H
#define UTILS_H

// other libs
#include <libxml/xmlreader.h>
#include <SDL_image.h>
#include <tmx.h>
// C++
#include <vector>
#include <sstream>
// C
#include <unistd.h>
#include <sys/time.h>


inline std::string extract_folder_from_full_path(const std::string & path) {
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos == std::string::npos)
    return "";
  return path.substr(0, 1 + slash_pos);
} // end extract_folder_from_full_path()

////////////////////////////////////////////////////////////////////////////////

class Timer {
public:
  typedef double Time;
  static const Time NOTIME = -1;
  Timer() { reset(); }
  virtual inline void reset() {
    gettimeofday(&start, NULL);
  }
  //! get the time since ctor or last reset (milliseconds)
  virtual inline Time getTimeSeconds() const {
    struct timeval end;
    gettimeofday(&end, NULL);
    return (Time) (// seconds
                   (end.tv_sec - start.tv_sec)
                   +
                   // useconds
                   (end.tv_usec - start.tv_usec)
                   / 1E6);
  }
private:
  struct timeval start;
}; // end class Timer

////////////////////////////////////////////////////////////////////////////////

class Rate {
public:
  Rate(double rate_hz) : _rate_hz(rate_hz) {
    _period_sec = 1. / _rate_hz;
  }

  double sleep() {
    double time_left = _period_sec - _timer.getTimeSeconds();
    if (time_left > 1E-3) // 1 ms
      usleep(1E6 * time_left);
    _timer.reset();
    return time_left;
  }

private:
  Timer _timer;
  double _rate_hz, _period_sec;
}; // end class Rate

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


// https://www.libsdl.org/release/SDL-1.2.15/docs/html/guidevideo.html
Uint32 getpixel(SDL_Surface *surface, int x, int y) {
  int bpp = surface->format->BytesPerPixel;
  /* Here p is the address to the pixel we want to retrieve */
  Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;
  switch(bpp) {
    case 1:
      return *p;
      break;

    case 2:
      return *(Uint16 *)p;
      break;

    case 3:
      if(SDL_BYTEORDER == SDL_BIG_ENDIAN)
        return p[0] << 16  |  p[1] << 8  |  p[2];
      else
        return p[0]  |  p[1] << 8  |  p[2] << 16;
      break;

    case 4:
      return *(Uint32 *)p;
      break;

    default:
      return 0;       /* shouldn't happen, but avoids warnings */
  }
}

////////////////////////////////////////////////////////////////////////////////

// https://www.libsdl.org/release/SDL-1.2.15/docs/html/guidevideo.html
void putpixel(SDL_Surface *surface, int x, int y, Uint32 pixel) {
  int bpp = surface->format->BytesPerPixel;
  /* Here p is the address to the pixel we want to set */
  Uint8 *p = (Uint8 *)surface->pixels + y * surface->pitch + x * bpp;

  switch(bpp) {
    case 1:
      *p = pixel;
      break;

    case 2:
      *(Uint16 *)p = pixel;
      break;

    case 3:
      if(SDL_BYTEORDER == SDL_BIG_ENDIAN) {
        p[0] = (pixel >> 16) & 0xff;
        p[1] = (pixel >> 8) & 0xff;
        p[2] = pixel & 0xff;
      } else {
        p[0] = pixel & 0xff;
        p[1] = (pixel >> 8) & 0xff;
        p[2] = (pixel >> 16) & 0xff;
      }
      break;

    case 4:
      *(Uint32 *)p = pixel;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////

// http://www.sdltutorials.com/sdl-scale-surface
SDL_Surface *ScaleSurface(SDL_Surface *Surface, Uint16 Width, Uint16 Height)
{
  if(!Surface  ||  !Width  ||  !Height)
    return 0;

  SDL_Surface *_ret = SDL_CreateRGBSurface(Surface->flags, Width, Height,
                                           Surface->format->BitsPerPixel,
                                           Surface->format->Rmask,
                                           Surface->format->Gmask,
                                           Surface->format->Bmask,
                                           Surface->format->Amask);


  double    _stretch_factor_x = (static_cast<double>(Width)  / static_cast<double>(Surface->w)),
      _stretch_factor_y = (static_cast<double>(Height) / static_cast<double>(Surface->h));

  for(Sint32 y = 0; y < Surface->h; y++)
    for(Sint32 x = 0; x < Surface->w; x++)
      for(Sint32 o_y = 0; o_y < _stretch_factor_y; ++o_y)
        for(Sint32 o_x = 0; o_x < _stretch_factor_x; ++o_x)
          putpixel(_ret, static_cast<Sint32>(_stretch_factor_x * x) + o_x,
                   static_cast<Sint32>(_stretch_factor_y * y) + o_y, getpixel(Surface, x, y));
  return _ret;
}

////////////////////////////////////////////////////////////////////////////////

// http://www.sdltutorials.com/sdl-scale-surface
SDL_Surface *FlipSurface(SDL_Surface *Surface)
{
  int w = Surface->w, h = Surface->h;
  if(!Surface || !w || !h)
    return 0;

  SDL_Surface *_ret = SDL_CreateRGBSurface(Surface->flags, w, h,
                                           Surface->format->BitsPerPixel,
                                           Surface->format->Rmask,
                                           Surface->format->Gmask,
                                           Surface->format->Bmask,
                                           Surface->format->Amask);

  for(Sint32 y = 0; y < h; y++)
    for(Sint32 x = 0; x < w; x++)
      putpixel(_ret, w-1-x, y, getpixel(Surface, x, y));
  return _ret;
}

////////////////////////////////////////////////////////////////////////////////

void render_polyline(SDL_Renderer* renderer,
                     double **points, double x, double y, int pointsc) {
  int i;
  for (i=1; i<pointsc; i++) {
    SDL_RenderDrawLine(renderer, x+points[i-1][0], y+points[i-1][1], x+points[i][0], y+points[i][1]);
  }
}

////////////////////////////////////////////////////////////////////////////////

void render_polygon(SDL_Renderer* renderer,
                    double **points, double x, double y, int pointsc) {
  render_polyline(renderer, points, x, y, pointsc);
  if (pointsc > 2) {
    SDL_RenderDrawLine(renderer, x+points[0][0], y+points[0][1], x+points[pointsc-1][0], y+points[pointsc-1][1]);
  }
}

////////////////////////////////////////////////////////////////////////////////

std::string rect2str(const SDL_Rect & r) {
  std::ostringstream out;
  out << "(" << r.x << ", " << r.y << ")+(" << r.w << ", " << r.h << ")";
  return out.str();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

unsigned int gid_clear_flags(unsigned int gid) {
  return gid & TMX_FLIP_BITS_REMOVAL;
}

////////////////////////////////////////////////////////////////////////////////

int tile_x(tmx_map  * map, unsigned int col, unsigned int row) {
  int retval = 0;
  switch (map->orient) {
    case O_STA: // staggered
      retval = col * map->tile_width;
      if (row % 2 == 1) // offset for oddlines
        retval += map->tile_width / 2;
      break;
    case O_ORT:
    default:
      retval = col * map->tile_width;
      break;
  }
  return retval;
}

////////////////////////////////////////////////////////////////////////////////

int tile_y(tmx_map  * map, unsigned int /*col*/, unsigned int row) {
  switch (map->orient) {
    case O_STA: // staggered
      return .5 * row * map->tile_height;
    case O_ORT:
    default:
      return row * map->tile_height;
  }
}

////////////////////////////////////////////////////////////////////////////////

/*!
 * \brief   detect if a point is inside a polygon - return true or false
 *  http://en.wikipedia.org/wiki/Point_in_polygon#Winding_number_algorithm
     *
 * \param   p the point
 * \param   poly the polygon
 * \return  true if the point is in the polygon
 */
template<class Point2>
static inline bool point_inside_polygon(const Point2 & p,
                                        const std::vector<Point2> & poly) {
  /*
     * algo from http://www.visibone.com/inpoly/
     */
  Point2 p_old, p_new, p1, p2;
  bool inside = false;
  int npoints = poly.size();
  if (npoints < 3) {
    return(0);
  }
  p_old = poly[npoints-1];

  for (int i=0 ; i < npoints ; i++) {
    p_new = poly[i];
    if (p_new.x > p_old.x) {
      p1 = p_old;
      p2 = p_new;
    }
    else {
      p1 = p_new;
      p2 = p_old;
    }
    if ((p_new.x < p.x) == (p.x <= p_old.x)          /* edge "open" at one end */
        && 1.f * (p.y-p1.y) * (p2.x-p1.x) < 1.f * (p2.y-p1.y) * (p.x-p1.x)) {
      inside = !inside;
    }
    p_old.x = p_new.x;
    p_old.y = p_new.y;
  } // end loop i
  return(inside);
}

void build_diamond_hittest(std::vector<bool> & ans, unsigned int w, unsigned int h) {
  std::vector<SDL_Point> pts(4);
  pts[0].x = w/2; pts[0].y = 0;
  pts[1].x = w;   pts[1].y = h/2;
  pts[2].x = w/2; pts[2].y = h;
  pts[3].x = 0;   pts[3].y = h/2;
  ans.resize(w * h, false);
  SDL_Point curr;
  for (unsigned int row = 0; row < h; ++row) {
    curr.y = row;
    for (unsigned int col = 0; col < w; ++col) {
      curr.x = col;
      if (point_inside_polygon(curr, pts))
        ans[col + row * w] = true;
      // printf("%c", ans[col + row * w] ? 'X' : '.');
    } // end for row
    //printf("\n");
  } // end for col
} // end build_diamond_hittest();

template <class _T>
void xy2tile_staggered(tmx_map  * map,
                       const _T & xscreen, const _T & yscreen,
                       int & col, int & row,
                       const std::vector<bool> & diamond_hittest) {
  // http://gamedev.stackexchange.com/questions/45103/staggered-isometric-map-calculate-map-coordinates-for-point-on-screen
  unsigned int tw = map->tile_width, th = map->tile_height;
  int diamondx = ((int) xscreen) % tw, diamondy = ((int) yscreen) % th;
  if (diamond_hittest[diamondx + diamondy * tw]) { /* On even tile */
    //printf("IN diamond:(%i, %i)\n", diamondx, diamondy);
    col= (int) ((xscreen + tw) / tw) - 1;
    row= 2 * ((int) ((yscreen + th) / th) - 1);
  }
  else { /* On odd tile */
    //printf("OUT diamond:(%i, %i)\n", diamondx, diamondy);
    col= (int) ((xscreen + tw / 2) / tw) - 1;
    row= 2 * ((int) ((yscreen + th / 2) / th)) - 1;
  }
}

////////////////////////////////////////////////////////////////////////////////

void set_color(SDL_Renderer* renderer,
               int color) {
  unsigned char r, g, b;
  r = (color >> 16) & 0xFF;
  g = (color >>  8) & 0xFF;
  b = (color)       & 0xFF;
  SDL_SetRenderDrawColor(renderer, r, g, b, SDL_ALPHA_OPAQUE);
}

////////////////////////////////////////////////////////////////////////////////

void render_objects(SDL_Renderer* renderer,
                    tmx_object_group *objgr) {
  SDL_Rect rect;
  //set_color(objgr->color);
  set_color(renderer, 0xFF0000);
  tmx_object *head = objgr->head;
  /* FIXME line thickness */
  while (head) {
    if (head->visible) {
      if (head->shape == S_SQUARE) {
        rect.x =     head->x;  rect.y =      head->y;
        rect.w = head->width;  rect.h = head->height;
        SDL_RenderDrawRect(renderer, &rect);
      } else if (head->shape  == S_POLYGON) {
        render_polygon(renderer, head->points, head->x, head->y, head->points_len);
      } else if (head->shape == S_POLYLINE) {
        render_polyline(renderer, head->points, head->x, head->y, head->points_len);
      } else if (head->shape == S_ELLIPSE) {
        /* FIXME: no function in SDL2 */
      }
    }
    head = head->next;
  }
}

////////////////////////////////////////////////////////////////////////////////

bool render_tile(SDL_Renderer* renderer,
                 tmx_map  * map, tmx_tile *tile,
                 unsigned int col, unsigned int row) {
  if (tile == NULL)
    return false;
  tmx_tileset *ts = tile->tileset;
  tmx_image *im = tile->image;
  SDL_Rect srcrect, dstrect;
  srcrect.x = tile->ul_x;
  srcrect.y = tile->ul_y;
  srcrect.w = dstrect.w = im->width;
  srcrect.h = dstrect.h = im->height;
  dstrect.x = tile_x(map, col, row);
  dstrect.y = tile_y(map, col, row) + map->tile_height - im->height;
  //  printf("(%i, %i) -> (%i, %i, %i, %i) - IM:(%li, %li)\n",
  //         col, row, dstrect.x, dstrect.y, dstrect.w, dstrect.h, im->width, im->height);
  /* TODO Opacity and Flips */
  SDL_Texture* tileset;
  if (im) {
    tileset = (SDL_Texture*)im->resource_image;
  }
  else {
    tileset = (SDL_Texture*)ts->image->resource_image;
  }
  return SDL_RenderCopy(renderer, tileset, &srcrect, &dstrect) == 0;
} // end render_tile();

////////////////////////////////////////////////////////////////////////////////

void render_layer(SDL_Renderer* _renderer,
                  tmx_map  * map, tmx_layer *layer) {
  for (unsigned int row=0; row < map->height; row++) {
    for (unsigned int col=0; col < map->width; col++) {
      unsigned int gid = gid_clear_flags(layer->content.gids[(row * map->width) + col]);
      render_tile(_renderer, map, map->tiles[gid], col, row);
    } // end loop col
  } // end loop row
} // end render_layer();

////////////////////////////////////////////////////////////////////////////////

void render_image_layer(SDL_Renderer* renderer,
                        tmx_image *img) {
  SDL_Rect dim;
  dim.x = dim.y = 0;
  SDL_QueryTexture((SDL_Texture*)img->resource_image, NULL, NULL, &(dim.w), &(dim.h));
  SDL_RenderCopy(renderer, (SDL_Texture*)img->resource_image, NULL, &dim);

}

////////////////////////////////////////////////////////////////////////////////

SDL_Texture* render_map(SDL_Renderer* renderer,
                        tmx_map  * map) {
  /* Bitmap's width and height */
  int w, h;
  switch (map->orient) {
    case O_STA: // staggered
      w = (map->width+.5) * map->tile_width;
      h = (map->height+2)/2 * map->tile_height;
      break;
    default:
      w = map->width  * map->tile_width;
      h = map->height * map->tile_height;
      break;
  }

  SDL_Texture *res;
  if (!(res = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, w, h))) {
    printf("Error while SDL_CreateTexture():'%s'\n", SDL_GetError());
    return NULL;
  }
  SDL_SetRenderTarget(renderer, res);

  set_color(renderer, map->backgroundcolor);
  SDL_RenderClear(renderer);
  tmx_layer *layer = map->ly_head;
  while (layer) {
    if (layer->visible) {
      if (layer->type == L_OBJGR) {
        render_objects(renderer, layer->content.objgr);
      } else if (layer->type == L_IMAGE) {
        render_image_layer(renderer, layer->content.image);
      } else if (layer->type == L_LAYER) {
        render_layer(renderer, map, layer);
      }
    }
    layer = layer->next;
  }
  set_color(renderer, 0x0);
  SDL_SetRenderTarget(renderer, NULL);
  return res;
}

////////////////////////////////////////////////////////////////////////////////

tmx_layer * get_layer(tmx_map* map, const std::string & layer_name) {
  tmx_layer *layer = map->ly_head;
  while (layer) {
    if (layer->name == layer_name)
      return layer;
    layer = layer->next;
  }
  printf("get_layer(): could not find layer '%s'\n", layer_name.c_str());
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////

template <class _T>
bool point_in_layer_staggered(tmx_map* map, tmx_layer* layer,
                              const _T & xscreen, const _T & yscreen,
                              const std::vector<bool> & diamond_hittest) {
  // transform pt to cell index
  int col, row;
  xy2tile_staggered(map, xscreen, yscreen, col, row, diamond_hittest);
  if (col < 0 || col >= (int) map->width || row < 0 || row >= (int) map->height)
    return false; // out of bounds
  // check cell index value
  unsigned int gid = gid_clear_flags(layer->content.gids[(row * map->width) + col]);
  //printf("point_in_layer: (%i, %i) -> tile (%i, %i) -> gid:%i\n", x, y, col, row, gid);
  return (gid);
}

#endif // UTILS_H
