#ifndef __RT4832_H__
#define __RT4832_H__

void rt4832_dsv_ctrl(int enable);
#if defined(CONFIG_LGD_INCELL_LG4894_HD_SF3)
extern int old_bl_level;
#endif
void rt4832_dsv_toggle_ctrl(void);
void rt4832_dsv_mode_change(int mode);

#endif
