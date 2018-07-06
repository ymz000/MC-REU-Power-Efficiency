/*
 * The Clear BSD License
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __static_queue__
#define __static_queue__

#include <stdlib.h>

/*!
 * @brief Base class which implements static queue as ring buffer that operates on data type void*.
 */

typedef struct
{
    uint32_t capacity;    //!< Capacity of queue
    void *data;           //!< Pointer to data of queue
    uint32_t head;        //!< Index to free slot
    uint32_t tail;        //!< Index to slot with data
    uint32_t elementSize; //!< Size of one element
} static_queue_t;

/*!
 * @brief This function initialize queue.
 *
 * @param[in] sq Structure of static queue.
 * @param[in] buffer Buffer used for storing elements.
 * @param[in] maxCapacity Capacity of queue.
 * @param[in] elementSize Size of one element in bytes.
 *
 * @return true The initialization was successful.
 * @return false The initialization was not successful.
 */
bool static_queue_init(static_queue_t *sq, uint8_t *buffer, uint32_t maxCapacity, uint32_t elementSize)
{
    if ((sq == NULL) || (buffer == NULL))
    {
        return false;
    }

    sq->data = NULL;
    sq->elementSize = elementSize;
    sq->capacity = maxCapacity;
    sq->head = 0;
    sq->tail = 0;

    sq->data = (void *)buffer;
    return true;
}

/*!
 * @brief Destructor of BaseStaticQueue class.
 *
 * This function free allocated buffer for data.
 */
void static_queue_deinit(static_queue_t *sq)
{
    free(sq->data);
}

/*!
 * @brief This function adds element to queue.
 *
 * @param[in] element Pointer to element for adding.
 *
 * @return true Element was added.
 * @return false Element was not added, queue is full.
 */
bool static_queue_add(static_queue_t *sq, void *element)
{
    if ((sq->head + 1) % sq->capacity != sq->tail)
    {
        memcpy((void *)((uint32_t)sq->data + sq->head * sq->elementSize), element, sq->elementSize);
        sq->head = (sq->head + 1) % sq->capacity;
        return true;
    }
    return false;
}

/*!
 * @brief This function returns pointer to element from queue.
 *
 * @return void* Pointer to element.
 */
void *static_queue_get(static_queue_t *sq)
{
    void *element = NULL;
    if (sq->tail != sq->head)
    {
        element = (void *)((uint32_t)sq->data + sq->tail * sq->elementSize);
        sq->tail = (sq->tail + 1) % sq->capacity;
    }
    return element;
}

/*!
* @brief This function returns number of elements in queue.
*
* @return int Number of elements in queue.
*/
int static_queue_size(static_queue_t *sq)
{
    if (sq->head >= sq->tail)
    {
        return sq->head - sq->tail;
    }
    return (sq->capacity - sq->tail + sq->head);
}

#endif // defined(__static_queue__)
