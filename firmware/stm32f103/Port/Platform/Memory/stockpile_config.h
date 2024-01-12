/******
	************************************************************************
	******
	** @versions : 1.1.4
	** @time     : 2020/09/15
	******
	************************************************************************
	******
	** @project : XDrive_Step
	** @brief   : 具有多功能接口和闭环功能的步进电机
	** @author  : unlir (知不知啊)
	******
	** @address : https://github.com/unlir/XDrive
	******
	** @issuer  : IVES ( 艾维斯 实验室) (QQ: 557214000)   (master)
	** @issuer  : REIN (  知驭  实验室) (QQ: 857046846)   (master)
	******
	************************************************************************
	******
	** {Stepper motor with multi-function interface and closed Main function.}
	** Copyright (c) {2020}  {unlir(知不知啊)}
	** 
	** This program is free software: you can redistribute it and/or modify
	** it under the terms of the GNU General Public License as published by
	** the Free Software Foundation, either version 3 of the License, or
	** (at your option) any later version.
	** 
	** This program is distributed in the hope that it will be useful,
	** but WITHOUT ANY WARRANTY; without even the implied warranty of
	** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	** GNU General Public License for more details.
	** 
	** You should have received a copy of the GNU General Public License
	** along with this program.  If not, see <http://www.gnu.org/licenses/>.
	******
	************************************************************************
******/

/*****
  ** @file     : stockpile_config.c/h
  ** @brief    : 存储配置
  ** @versions : newest
  ** @time     : newest
  ** @reviser  : unli (HeFei China)
  ** @explain  : null
*****/

/*************************************************************** Stockpile_Start ***************************************************************/

#ifndef STOCKPILE_CONFIG_H
#define STOCKPILE_CONFIG_H

#include "main.h"

//APP_FIRMWARE
#define		STOCKPILE_APP_FIRMWARE_ADDR			(0x08000000) //(0x0800C000)		//起始地址
#define		STOCKPILE_APP_FIRMWARE_SIZE			(1024*47)		//Flash容量    47K    XDrive(APP_FIRMWARE)
//APP_CALI
#if REDUCE_RESOLUTION
#define		STOCKPILE_APP_CALI_ADDR					(STOCKPILE_APP_FIRMWARE_ADDR+STOCKPILE_APP_FIRMWARE_SIZE)		//起始地址
#define		STOCKPILE_APP_CALI_SIZE					(1024*16)		//Flash容量    16K    XDrive(APP_CALI)(可容纳16K-2byte校准数据-即最大支持14位编码器的校准数据)
#else
#define		STOCKPILE_APP_CALI_ADDR					(STOCKPILE_APP_FIRMWARE_ADDR+STOCKPILE_APP_FIRMWARE_SIZE)		//起始地址
#define		STOCKPILE_APP_CALI_SIZE					(1024*32)		//Flash容量    32K    XDrive(APP_CALI)(可容纳16K-2byte校准数据-即最大支持14位编码器的校准数据)
#endif
//APP_DATA
#define		STOCKPILE_APP_DATA_ADDR					(STOCKPILE_APP_CALI_ADDR + STOCKPILE_APP_CALI_SIZE)		//起始地址
#define		STOCKPILE_APP_DATA_SIZE					(1024)		//Flash容量     1K    XDrive(APP_DATA)

#endif
