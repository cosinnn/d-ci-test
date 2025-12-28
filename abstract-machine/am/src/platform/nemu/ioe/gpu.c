#include <am.h>
#include <nemu.h>

#define SYNC_ADDR (VGACTL_ADDR + 4)

void __am_gpu_init() {
  int i;
  uint32_t wh = inl(VGACTL_ADDR);   // 读屏幕宽高
  int w = wh >> 16;                 // 高16位: 宽
  int h = wh & 0xffff; 
  uint32_t *fb = (uint32_t *)(uintptr_t)FB_ADDR;
  for (i = 0; i < w * h; i ++) fb[i] = i;
  outl(SYNC_ADDR, 1);
}

void __am_gpu_config(AM_GPU_CONFIG_T *cfg) {
  uint32_t screen_size = inl(VGACTL_ADDR);
  int width = (screen_size >> 16) & 0xffff;
  int height = (screen_size) & 0xffff;
  *cfg = (AM_GPU_CONFIG_T) {
    .present = true, .has_accel = false,
    .width = width, .height = height,
    .vmemsz = 0
  };
}

void __am_gpu_fbdraw(AM_GPU_FBDRAW_T *ctl) {
  int x = ctl->x,y = ctl->y,w = ctl->w,h = ctl->h;
  uint32_t *pixels = ctl->pixels;// 帧缓冲区数据
  uint32_t screen_wdh = inl(VGACTL_ADDR) >> 16;
  uint32_t *vmem = (uint32_t *)(uintptr_t)FB_ADDR;
  // 计算需要绘制的区域大小
  for (int j = y; j < y+h; j++) {
    for (int i = x; i < x+w; i++) {
      int pixel_index = j * screen_wdh + i;
      vmem[pixel_index] = pixels[w*(j-y)+(i-x)];  // 将帧缓冲区的数据写入 vmem
    }
  }
  if (ctl->sync) {
    outl(SYNC_ADDR, 1);
  }
}

void __am_gpu_status(AM_GPU_STATUS_T *status) {
  status->ready = true;
}
