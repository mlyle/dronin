/**
 ******************************************************************************
 * @addtogroup TauLabsCore Tau Labs Core components
 * @{
 * @addtogroup UAVObjects UAVObject set for this firmware
 * @{
 *
 * @file       $(NAMELC).c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2015
 * @author     dRonin, http://dronin.org Copyright (C) 2015
 * @brief      Implementation of the $(NAME) object. This file has been 
 *             automatically generated by the UAVObjectGenerator.
 * 
 * @note       Object definition file: $(XMLFILE). 
 *             This is an automatically generated file.
 *             DO NOT modify manually.
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Additional note on redistribution: The copyright and license notices above
 * must be maintained in each individual source file that is a derivative work
 * of this source file; otherwise redistribution is prohibited.
 */

#include <string.h>
#include "uavobjectmanager.h"
#include "$(NAMELC).h"

// Private variables
static UAVObjHandle handle = NULL;

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t $(NAME)Initialize(void)
{
	// Don't set the handle to null if already registered
	if(UAVObjGetByID($(NAMEUC)_OBJID) != NULL)
		return -2;
	
	// Register object with the object manager
	handle = UAVObjRegister($(NAMEUC)_OBJID,
			$(NAMEUC)_ISSINGLEINST, $(NAMEUC)_ISSETTINGS, $(NAMEUC)_NUMBYTES, &$(NAME)SetDefaults);

	// Done
	if (handle != 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

static void $(NAME)SetDefaultsImpl(UAVObjHandle obj, uint16_t instId) {
	// Initialize object fields to their default values
	$(NAME)Data data = {};

$(INITFIELDS)
	UAVObjSetInstanceData(obj, instId, &data);
}

static void $(NAME)SetMetadataDefaults(UAVObjHandle obj) {
	// Initialize object metadata to their default values
	UAVObjMetadata metadata;
	metadata.flags =
		$(FLIGHTACCESS) << UAVOBJ_ACCESS_SHIFT |
		$(GCSACCESS) << UAVOBJ_GCS_ACCESS_SHIFT |
		$(FLIGHTTELEM_ACKED) << UAVOBJ_TELEMETRY_ACKED_SHIFT |
		$(GCSTELEM_ACKED) << UAVOBJ_GCS_TELEMETRY_ACKED_SHIFT |
		$(FLIGHTTELEM_UPDATEMODE) << UAVOBJ_TELEMETRY_UPDATE_MODE_SHIFT |
		$(GCSTELEM_UPDATEMODE) << UAVOBJ_GCS_TELEMETRY_UPDATE_MODE_SHIFT;
	metadata.telemetryUpdatePeriod = $(FLIGHTTELEM_UPDATEPERIOD);
	metadata.gcsTelemetryUpdatePeriod = $(GCSTELEM_UPDATEPERIOD);
	metadata.loggingUpdatePeriod = $(LOGGING_UPDATEPERIOD);
	UAVObjSetMetadata(obj, &metadata);
}

/**
 * Initialize object fields and metadata with the default values.
 * If a default value is not specified the object fields
 * will be initialized to zero.
 */
void $(NAME)SetDefaults(UAVObjHandle obj, uint16_t instId)
{
	$(NAME)SetDefaultsImpl(obj, instId);

	if (instId == 0) {
		$(NAME)SetMetadataDefaults(obj);
	}
}

/**
 * Get object handle
 */
UAVObjHandle $(NAME)Handle()
{
	return handle;
}

/**
 * Get/Set object Functions
 */
$(SETGETFIELDS)

/**
 * @}
 * @}
 */

