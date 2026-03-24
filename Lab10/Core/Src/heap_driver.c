#include "Lab10_heap_driver.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)

/* Block map: 0 = free, 1 = used */
static uint8_t block_map[BLOCK_COUNT];

/* ================= heap_init ================= */
void heap_init(void)
{
    // Set all blocks to free
    memset(block_map, 0, sizeof(block_map));
}

/* ================= heap_alloc ================= */
void* heap_alloc(size_t size)
{
    if (size == 0)
        return NULL;

    // Calculate required number of blocks
    size_t needed = (size + BLOCK_SIZE - 1) / BLOCK_SIZE;

    size_t count = 0;
    size_t start = 0;

    // Search for contiguous free blocks
    for (size_t i = 0; i < BLOCK_COUNT; i++)
    {
        if (block_map[i] == 0)
        {
            if (count == 0)
                start = i;

            count++;

            if (count == needed)
            {
                // Mark blocks as used
                for (size_t j = start; j < start + needed; j++)
                {
                    block_map[j] = 1;
                }

                // Return pointer to start address
                return (void*)(HEAP_START_ADDR + (start * BLOCK_SIZE));
            }
        }
        else
        {
            count = 0;  // reset if block is occupied
        }
    }

    // No space available
    return NULL;
}

/* ================= heap_free ================= */
void heap_free(void* ptr)
{
    if (ptr == NULL)
        return;

    uint8_t* addr = (uint8_t*)ptr;
    

    // Check if pointer is within heap range
    if (addr < HEAP_START_ADDR || addr >= (HEAP_START_ADDR + HEAP_SIZE))
        return;
    // Check if pointer is aligned to block boundary
    if ((addr - HEAP_START_ADDR) % BLOCK_SIZE != 0)
        return;

    // Calculate block index
    size_t index = (addr - HEAP_START_ADDR) / BLOCK_SIZE;

    // Free blocks until a free block is found
    for (size_t i = index; i < BLOCK_COUNT; i++)
    {
        if (block_map[i] == 1)
        {
            block_map[i] = 0;
        }
        else
        {
            break;  // stop when already free
        }
    }
}