/*
 * include/video/omaplfb-dev.h
 *
 * Device interface to OMAP SGX display driver
 *
 * Copyright (C) 2011 Texas Instruments, Inc
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

#ifndef __OMAPLFB_DEV_H__
#define __OMAPLFB_DEV_H__

struct omaplfb_clone_cmd {
	int mgr_id_src;
	int mgr_id_dst;
};

struct omaplfb_dsscomp_info {
	int length;
	void *composition;
};

#define OMAPLFB_DEV_NAME "omaplfb"

#define OMAPLFB_CLONING_ENABLE _IOW('O', 128, struct omaplfb_clone_cmd)
#define OMAPLFB_CLONING_DISABLE _IOW('O', 129, struct omaplfb_clone_cmd)
#define OMAPLFB_DSSCOMP_SETUP _IOW('O', 130, struct omaplfb_dsscomp_info)

#endif /* __OMAPLFB_DEV_H__ */


