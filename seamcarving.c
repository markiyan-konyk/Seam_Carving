#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "seamcarving.h"

void calc_energy(struct rgb_img *im, struct rgb_img **grad) {
    size_t height = im->height;
    size_t width = im->width;
    
    create_img(grad, height, width);  

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            size_t left_x = (x == 0) ? (width - 1) : (x - 1);
            size_t right_x = (x == width - 1) ? 0 : (x + 1);
            size_t up_y = (y == 0) ? (height - 1) : (y - 1);
            size_t down_y = (y == height - 1) ? 0 : (y + 1);
            
            int Rx = get_pixel(im, y, right_x, 0) - get_pixel(im, y, left_x, 0);
            int Gx = get_pixel(im, y, right_x, 1) - get_pixel(im, y, left_x, 1);
            int Bx = get_pixel(im, y, right_x, 2) - get_pixel(im, y, left_x, 2);
            int delta_x2 = Rx * Rx + Gx * Gx + Bx * Bx;

            int Ry = get_pixel(im, down_y, x, 0) - get_pixel(im, up_y, x, 0);
            int Gy = get_pixel(im, down_y, x, 1) - get_pixel(im, up_y, x, 1);
            int By = get_pixel(im, down_y, x, 2) - get_pixel(im, up_y, x, 2);
            int delta_y2 = Ry * Ry + Gy * Gy + By * By;

            double energy = sqrt(delta_x2 + delta_y2);
            uint8_t energy_val = (uint8_t)(energy / 10);  
            
            set_pixel(*grad, y, x, energy_val, energy_val, energy_val);
        }
    }
}

void dynamic_seam(struct rgb_img *grad, double **best_arr) {
    size_t height = grad->height;
    size_t width = grad->width;

    *best_arr = (double *)malloc(height * width * sizeof(double));
    if (*best_arr == NULL) {
        fprintf(stderr, "Memory allocation failed for best_arr\n");
        exit(1);
    }

    for (size_t x = 0; x < width; x++) {
        (*best_arr)[x] = (double)get_pixel(grad, 0, x, 0);
    }

    for (size_t y = 1; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            double min_prev = (*best_arr)[(y - 1) * width + x];

            if (x > 0) {
                min_prev = fmin(min_prev, (*best_arr)[(y - 1) * width + (x - 1)]);
            }

            if (x < width - 1) {
                min_prev = fmin(min_prev, (*best_arr)[(y - 1) * width + (x + 1)]);
            }

            (*best_arr)[y * width + x] = (double)get_pixel(grad, y, x, 0) + min_prev;
        }
    }
}

void recover_path(double *best, int height, int width, int **path) {
    
    *path = (int *)malloc(height * sizeof(int));
    if (*path == NULL) {
        fprintf(stderr, "Memory allocation failed for path\n");
        exit(1);
    }

    int min_index = 0;
    double min_energy = best[(height - 1) * width];

    for (int x = 1; x < width; x++) {
        if (best[(height - 1) * width + x] < min_energy) {
            min_energy = best[(height - 1) * width + x];
            min_index = x;
        }
    }

    (*path)[height - 1] = min_index; 

    for (int y = height - 2; y >= 0; y--) {
        int prev_x = (*path)[y + 1]; 
        int best_x = prev_x;         
        double best_value = best[y * width + prev_x];

        if (prev_x > 0 && best[y * width + (prev_x - 1)] < best_value) {
            best_x = prev_x - 1;
            best_value = best[y * width + best_x];
        }

        if (prev_x < width - 1 && best[y * width + (prev_x + 1)] < best_value) {
            best_x = prev_x + 1;
        }

        (*path)[y] = best_x;
    }
}

void remove_seam(struct rgb_img *src, struct rgb_img **dest, int *path) {
    if (src == NULL || dest == NULL || path == NULL) {
        return;
    }
   
    if (src->width <= 0) {
        return;
    }

    size_t new_width = src->width - 1;
    
    create_img(dest, src->height, new_width);

    for (size_t y = 0; y < src->height; y++) {
        size_t dest_x = 0;
        for (size_t x = 0; x < src->width; x++) {
            if ((int)x != path[y]) {
                uint8_t r = get_pixel(src, y, x, 0);
                uint8_t g = get_pixel(src, y, x, 1);
                uint8_t b = get_pixel(src, y, x, 2);
                
                set_pixel(*dest, y, dest_x, r, g, b);
                dest_x++;
            }
        }
    }
}

int main() {

    struct rgb_img *im;
    struct rgb_img *cur_im;
    struct rgb_img *grad;
    double *best;
    int *path;

    read_in_img(&im, "image.bin");
    
    for(int i = 0; i < 200; i++){
        printf("i = %d\n", i);
        calc_energy(im,  &grad);
        dynamic_seam(grad, &best);
        recover_path(best, grad->height, grad->width, &path);
        remove_seam(im, &cur_im, path);

        destroy_image(im);
        destroy_image(grad);
        free(best);
        free(path);
        im = cur_im;
    }
    char filename[200];
    sprintf(filename, "output.bin");
    write_img(cur_im, filename);
    destroy_image(im);

    return 0;
}


