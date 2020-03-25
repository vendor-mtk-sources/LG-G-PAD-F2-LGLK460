/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/*
 * DW9714AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "DW9714AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif
//[LGE_UPDATE_S] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
typedef enum
{
    AF_disabled,
    AF_initializing,
    AF_driving,
    AF_releasing
}STATUS_NUM;
//[LGE_UPDATE_E] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
static struct i2c_client *g_pstAF_I2Cclient;
static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static unsigned long g_u4AF_INF;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;
static char status_AF; //[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
static char is_register_setted;

#if 0
static int s4AF_ReadReg(unsigned short *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff[2];

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff, 2);

	if (i4RetValue < 0) {
		LOG_INF("I2C read failed!!\n");
		return -1;
	}

	*a_pu2Result = (((u16) pBuff[0]) << 4) + (pBuff[1] >> 4);

	return 0;
}
#endif

static int s4AF_WriteReg(u16 a_u2Data, unsigned char delay)
{
	int i4RetValue = 0;
	char puSendCmd[2] = { (char)((a_u2Data & 0xFF00) >> 8), (char)((a_u2Data & 0xFF)) };
	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

	i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);
    msleep(delay);

	if (i4RetValue < 0) {
		LOG_INF("I2C send failed!!\n");
		return -1;
	}
	return 0;
}
static inline int getAFInfo(__user stAF_MotorInfo *pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

	stMotorInfo.u4MacroPosition = g_u4AF_MACRO;
	stMotorInfo.u4InfPosition = g_u4AF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = 1;

	stMotorInfo.bIsMotorMoving = 1;

	if (*g_pAF_Opened >= 1)
		stMotorInfo.bIsMotorOpen = 1;
	else
		stMotorInfo.bIsMotorOpen = 0;

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stAF_MotorInfo)))
		LOG_INF("copy to user failed when getting motor information\n");

	return 0;
}

static inline int moveAF(unsigned long a_u4Position)
{
	//int ret = 0;
    unsigned long a_u2Data = 0;
	unsigned char delay = 0;
	if ((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF)) {
		LOG_INF("out of range\n");
		return -EINVAL;
	}

    if (*g_pAF_Opened == 1) {
        status_AF = AF_initializing;//[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
        spin_lock(g_pAF_SpinLock);
        *g_pAF_Opened = 2;
        spin_unlock(g_pAF_SpinLock);
    }
	if (g_u4CurrPosition == a_u4Position && status_AF != AF_initializing)
		return 0;

	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);

	/* LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */

    //[LGE_UPDATE_S] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
    a_u2Data = ((unsigned long)g_u4TargetPosition << 4); // AF is moving with direct mode when AF is driving
    if(status_AF == AF_initializing)
    {
        printk("[LGE]_dw9714_set_initializing_mode\n");
        s4AF_WriteReg(0xECA3, 1); // Protection OFF
        s4AF_WriteReg(0xA106, 1); // direct & linear slope mode, MCLK = half
        s4AF_WriteReg(0xF280, 1); // T_SRC[4:0] = 0b10000 (136us per step)
        s4AF_WriteReg(0xDC51, 1); // Prptection On
        s4AF_WriteReg(0x12c5, 70); // move to position 300 in decimal
        a_u2Data |= 0x5; // AF is moving with linear slope mode when AF is initializing
        is_register_setted = 0;
        if(g_u4TargetPosition == 0)
        {
            status_AF = AF_driving;
            return 0;
        }
    }
    else if(status_AF == AF_releasing)
    {
        printk("[LGE]_dw9714_set_releasing_mode\n");
        s4AF_WriteReg(0xECA3, 1); // Protection OFF
        s4AF_WriteReg(0xA106, 1); // direct & linear slope mode, MCLK = half
        s4AF_WriteReg(0xF280, 1); // T_SRC[4:0] = 0b10000 (136us per step)
        s4AF_WriteReg(0xDC51, 1); // Prptection On
        s4AF_WriteReg(0x646, 70); // move to position 100 in decimal
        a_u2Data |= 0x6; // AF is moving with linear slope mode when AF is releasing
        is_register_setted = 0;
    }
    else if(!is_register_setted)
    {
        printk("[LGE]_dw9714_set_driving_mode\n");
        s4AF_WriteReg(0xECA3, 1); // Protection OFF
        s4AF_WriteReg(0xA105, 1); // direct & linear slope mode, MCLK = default
        s4AF_WriteReg(0xDC51, 1); // Prptection On
        is_register_setted = 1;
    }
    //[LGE_UPDATE_E] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
    if(status_AF ==AF_releasing || status_AF == AF_initializing)
        delay = 70;
	if (s4AF_WriteReg(a_u2Data, delay) == 0) {
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	} else {
		LOG_INF("set I2C failed when moving the motor\n");
	}
    if(status_AF == AF_initializing)
        status_AF = AF_driving;//[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time

	return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_INF = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
	spin_lock(g_pAF_SpinLock);
	g_u4AF_MACRO = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
long DW9714AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case AFIOC_G_MOTORINFO:
		i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
		break;

	case AFIOC_T_MOVETO:
		i4RetValue = moveAF(a_u4Param);
		break;

	case AFIOC_T_SETINFPOS:
		i4RetValue = setAFInf(a_u4Param);
		break;

	case AFIOC_T_SETMACROPOS:
		i4RetValue = setAFMacro(a_u4Param);
		break;

	default:
		LOG_INF("No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
int DW9714AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	LOG_INF("Start\n");
    status_AF = AF_releasing;//[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time

	if (*g_pAF_Opened) {
		LOG_INF("Free\n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
	}

    if (*g_pAF_Opened == 0)
    {
        moveAF(0);
        status_AF = AF_disabled;//[LGE_UPDATE] [kyunghun.oh@lge.com] [2017-04-20] Slope Controlling for reducing tick noise when open and release time
        s4AF_WriteReg(0x8000, 1);
    }
	LOG_INF("End\n");

	return 0;
}

void DW9714AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;
}
