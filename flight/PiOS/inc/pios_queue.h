/**
 ******************************************************************************
 * @file       pios_queue.h
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2014
 * @author     dRonin, http://dronin.org Copyright (C) 2015
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_Queue Queue Abstraction
 * @{
 * @brief Abstracts the concept of a queue to hide different implementations
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
 * with this program; if not, see <http://www.gnu.org/licenses/>
 */

#ifndef PIOS_QUEUE_H_
#define PIOS_QUEUE_H_

#define PIOS_QUEUE_TIMEOUT_MAX 0xffffffff

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * The following functions implement the concept of a queue usable
 * with PIOS_INCLUDE_CHIBIOS.
 *
 * for details see
 * http://www.freertos.org/a00018.html
 * http://chibios.sourceforge.net/html/group__mailboxes.html
 *
 */

struct pios_queue *PIOS_Queue_Create(size_t queue_length, size_t item_size);
void PIOS_Queue_Delete(struct pios_queue *queuep);
bool PIOS_Queue_Send(struct pios_queue *queuep, const void *itemp, uint32_t timeout_ms);
bool PIOS_Queue_Send_FromISR(struct pios_queue *queuep, const void *itemp, bool *wokenp);
bool PIOS_Queue_Receive(struct pios_queue *queuep, void *itemp, uint32_t timeout_ms);
size_t PIOS_Queue_GetItemSize(struct pios_queue *queuep);

#endif /* PIOS_QUEUE_H_ */

/**
  * @}
  * @}
  */
