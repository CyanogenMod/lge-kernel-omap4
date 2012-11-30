/*
 * LGE board common header file.
 *
 * Copyright (C) 2010 LG Electronics, Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LGE_PLAT_BOARD_H
#define __LGE_PLAT_BOARD_H

#if defined(CONFIG_MACH_LGE_IFF)
#include <lge/board/iff.h>
#endif

#if defined(CONFIG_MACH_LGE_P2)
#include <lge/board/p2.h>
#elif defined(CONFIG_MACH_LGE_U2)
#include <lge/board/u2.h>
#endif

#if defined(CONFIG_MACH_LGE_HDK)
#include <lge/board/lghdk.h>
#endif

#if defined(CONFIG_MACH_LGE_COSMO)
#include <lge/board/cosmo.h>
#endif

#endif /* __LGE_PLAT_BOARD_H */
