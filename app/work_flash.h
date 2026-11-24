#ifndef ROUND_FOC_WORK_FLASH_H
#define ROUND_FOC_WORK_FLASH_H

#include <stdbool.h>
#include <stdint.h>

/* Word offsets within two independently erasable Work Flash sectors. */
typedef enum { WF_OK, WF_INVALID, WF_IO_ERROR } FlashRead;
FlashRead work_flash_read(unsigned slot, unsigned offset, uint32_t *words, unsigned count);
bool work_flash_erase(unsigned slot);
bool work_flash_program(unsigned slot, unsigned offset, uint32_t word);

#endif
