/*
 * arch/arm/plat-omap/include/plat/dsscomp.h
 *
 * DSS Composition basic operation support
 *
 * Copyright (C) 2011 Texas Instruments, Inc
 * Author: Lajos Molnar <molnar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _ARCH_ARM_PLAT_OMAP_DSSCOMP_H
#define _ARCH_ARM_PLAT_OMAP_DSSCOMP_H

#include <plat/display.h>

/* queuing operations */
typedef struct dsscomp_data *dsscomp_t;		/* handle */

dsscomp_t dsscomp_new_sync_id(struct omap_overlay_manager *mgr, u32 sync_id,
			bool blankpost);
u32 dsscomp_first_sync_id(struct omap_overlay_manager *mgr);
dsscomp_t dsscomp_find(struct omap_overlay_manager *mgr, u32 sync_id);
u32 dsscomp_get_ovls(dsscomp_t comp);
int dsscomp_set_ovl(dsscomp_t comp, struct dss2_ovl_info *ovl);
int dsscomp_get_ovl(dsscomp_t comp, u32 ix, struct dss2_ovl_info *ovl);
int dsscomp_get_first_ovl(dsscomp_t comp, struct dss2_ovl_info *ovl);
int dsscomp_set_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr);
void dsscomp_prepdata(struct dsscomp_setup_mgr_data *d);
void dsscomp_prepdata_drop(struct dsscomp_setup_mgr_data *d);
dsscomp_t dsscomp_createcomp(struct omap_overlay_manager *mgr,
			struct dsscomp_setup_mgr_data *d, bool blankpost);
int dsscomp_get_mgr(dsscomp_t comp, struct dss2_mgr_info *mgr);
int dsscomp_setup(dsscomp_t comp, enum dsscomp_setup_mode mode,
			struct dss2_rect_t win);
int dsscomp_apply(dsscomp_t comp);
int dsscomp_wait(dsscomp_t comp, enum dsscomp_wait_phase phase, int timeout);
void dsscomp_drop(dsscomp_t c);
void dsscomp_release_active_comps(void);

#endif
