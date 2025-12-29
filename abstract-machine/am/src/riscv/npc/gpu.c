#include <am.h>
#include "../riscv.h"
#include <stdio.h>
#define VGACTL_ADDR 0xa0000100
#define SYNC_ADDR (VGACTL_ADDR + 4)
#define FB_ADDR 0xa1000000
void __am_gpu_init() {
//   int i;
//   int w = 600;  // TODO: get the correct width
//   int h = 800;  // TODO: get the correct height
//   uint32_t *fb = (uint32_t *)(uintptr_t)FB_ADDR;
//   for (i = 0; i < w * h; i ++) fb[i] = i;
//   outl(SYNC_ADDR, 1);
}

void __am_gpu_config(AM_GPU_CONFIG_T *cfg) {
  // uint32_t wh = inb(VGACTL_ADDR);
  // uint32_t wl = inb(VGACTL_ADDR+1);
  // uint32_t hh = inb(VGACTL_ADDR+2);
  // uint32_t hl = inb(VGACTL_ADDR+3);
  // uint32_t size =hl<<24|hh<<16|wl<<8|wh;
  // printf("size:%d\n",size);
  // *cfg = (AM_GPU_CONFIG_T) {
  //   .present = true, .has_accel = false,
  //   .width = size>>16, .height = size&0xffff,
  //   .vmemsz = 0
  // };
}

void __am_gpu_fbdraw(AM_GPU_FBDRAW_T *ctl) {
  // int x = ctl->x;
  // int y = ctl->y;
  // int w = ctl->w;
  // int h = ctl->h; 
  // if (!ctl->sync && (w == 0 || h == 0)) 
  // {
  //   return;
  // }
  // uint32_t wh = inb(VGACTL_ADDR);
  // uint32_t wl = inb(VGACTL_ADDR+1);
  // uint32_t hh = inb(VGACTL_ADDR+2);
  // uint32_t hl = inb(VGACTL_ADDR+3);
  // uint32_t size =hl<<24|hh<<16|wl<<8|wh;
  // uint32_t *pixels = ctl->pixels;
  // uint32_t *fb = (uint32_t *)(uintptr_t)FB_ADDR;
  // for(int i = y;i<y+h;i++)
  // {
  //   for(int j = x;j<x+w;j++)
  //   {
  //     fb[(size>>16)*i+j] = pixels[w*(i-y)+(j-x)];
  //   }
  // }
  // if (ctl->sync) {
  //   outl(SYNC_ADDR, 1);
  // }
}

void __am_gpu_status(AM_GPU_STATUS_T *status) {
  // status->ready = true;
}
