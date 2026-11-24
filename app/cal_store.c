#include "cal_store.h"
#include "work_flash.h"
#include <string.h>

enum { FORMAT, SEQUENCE, DIRECTION, ZERO, CRC, COMMIT, WORD_COUNT };
/* Changing the motor, angle convention or commutation table requires a new format. */
#define CAL_FORMAT  (0x52460100u | MOTOR_POLE_PAIRS)
#define CAL_COMMIT  0x43414C21u

static uint32_t record_crc(const uint32_t words[])
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < CRC; ++i) {
        crc ^= words[i];
        for (unsigned bit = 0; bit < 32; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xEDB88320u : 0u);
        }
    }
    return ~crc;
}

static CalRead read_slot(unsigned slot, uint32_t words[WORD_COUNT])
{
    uint32_t marker;
    FlashRead result = work_flash_read(slot, COMMIT, &marker, 1);
    if (result == WF_IO_ERROR) return CAL_IO_ERROR;
    if (result != WF_OK || marker != CAL_COMMIT) return CAL_EMPTY;
    result = work_flash_read(slot, 0, words, COMMIT);
    if (result == WF_IO_ERROR) return CAL_IO_ERROR;
    if (result != WF_OK || words[FORMAT] != CAL_FORMAT || words[DIRECTION] > 1u ||
        words[ZERO] >= ENCODER_COUNTS || words[CRC] != record_crc(words)) return CAL_EMPTY;
    return CAL_VALID;
}

static CalRead latest(unsigned *slot, uint32_t *sequence, Calibration *cal, bool for_write)
{
    CalRead found = CAL_EMPTY;
    bool io_error = false;
    for (unsigned i = 0; i < 2; ++i) {
        uint32_t words[WORD_COUNT];
        CalRead status = read_slot(i, words);
        if (status == CAL_IO_ERROR) io_error = true;
        if (status == CAL_VALID &&
            (found == CAL_EMPTY || (words[SEQUENCE] - *sequence < 0x80000000u))) {
            found = CAL_VALID;
            *slot = i;
            *sequence = words[SEQUENCE];
            cal->direction = words[DIRECTION] ? -1 : 1;
            cal->zero_electrical = (uint16_t)words[ZERO];
        }
    }
    /* A known-good copy is safe to use. Replacing a record requires knowing
       which sector is safe to erase, even when the other copy looks good. */
    if (io_error && (for_write || found == CAL_EMPTY)) return CAL_IO_ERROR;
    return found;
}

CalRead cal_store_load(Calibration *cal)
{
    unsigned slot = 0;
    uint32_t sequence = 0;
    return latest(&slot, &sequence, cal, false);
}

bool cal_store_save(const Calibration *cal)
{
    unsigned active = 1;
    uint32_t sequence = 0;
    Calibration previous;
    if ((cal->direction != 1 && cal->direction != -1) || cal->zero_electrical >= ENCODER_COUNTS ||
        latest(&active, &sequence, &previous, true) == CAL_IO_ERROR) return false;

    unsigned target = 1u - active;
    uint32_t words[WORD_COUNT] = { CAL_FORMAT, sequence + 1u, cal->direction < 0,
                                   cal->zero_electrical, 0, CAL_COMMIT };
    uint32_t verified[WORD_COUNT];
    words[CRC] = record_crc(words);
    if (!work_flash_erase(target)) return false;
    for (unsigned i = 0; i < COMMIT; ++i) {
        if (!work_flash_program(target, i, words[i])) return false;
    }
    if (work_flash_read(target, 0, verified, COMMIT) != WF_OK ||
        memcmp(words, verified, COMMIT * sizeof(uint32_t)) != 0) return false;

    /* A reset before this last word leaves the previous sector authoritative. */
    return work_flash_program(target, COMMIT, CAL_COMMIT) &&
           read_slot(target, verified) == CAL_VALID &&
           memcmp(words, verified, COMMIT * sizeof(uint32_t)) == 0;
}
