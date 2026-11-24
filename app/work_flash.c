#include "work_flash.h"
#include "cy_project.h"
#include "flash/cy_flash.h"
#include "dma/cy_mdma.h"

/* IAR reserves both sectors in the link map without emitting download data. */
__root __no_init static uint32_t slots[2][CY_WORK_SES_SIZE_IN_WORD] @ CY_WFLASH_SM_SBM_BASE;

static bool in_slot(unsigned slot, unsigned offset, unsigned count)
{
    return slot < 2 && count > 0 && count <= CY_WORK_SES_SIZE_IN_WORD &&
           offset <= CY_WORK_SES_SIZE_IN_WORD - count &&
           Cy_Flashc_GetWorkBankMode() == CY_FLASH_SINGLE_BANK_MODE;
}

static cy_en_flashdrv_status_t blank_check(uint32_t *address, unsigned count)
{
    const cy_stc_flash_blankcheck_config_t config = {
        .addrToBeChecked = address, .numOfWordsToBeChecked = count
    };
    return Cy_Flash_BlankCheck(NULL, &config, CY_FLASH_DRIVER_BLOCKING);
}

FlashRead work_flash_read(unsigned slot, unsigned offset, uint32_t *words, unsigned count)
{
    if (!in_slot(slot, offset, count)) return WF_IO_ERROR;
    uint32_t *source = &slots[slot][offset];
    cy_en_flashdrv_status_t status = blank_check(source, count);
    if (status == CY_FLASH_DRV_SUCCESS) return WF_INVALID;
    if (status != CY_FLASH_DRV_FLASH_NOT_ERASED) return WF_IO_ERROR;

    /* CYT2BL erratum 137: let AHB M-DMA report an ECC bus error on a torn word.
       A CPU load from that word could fault before the other slot is examined. */
    static cy_stc_mdma_descr_t descriptor;
    const cy_stc_mdma_descr_config_t transfer = {
        .intrType = CY_MDMA_INTR_DESCR_CMPLT, .chStateAtCmplt = CY_MDMA_CH_DISABLED,
        .triginType = CY_MDMA_TRIGIN_DESCR, .dataSize = CY_MDMA_WORD,
        .descrType = CY_MDMA_1D_TRANSFER, .srcAddr = source, .destAddr = words,
        .srcXincr = 1, .destXincr = 1, .xCount = count
    };
    const cy_stc_mdma_chnl_config_t channel = { .MDMA_Descriptor = &descriptor };
    Cy_MDMA_Chnl_Disable(DMAC, 0);
    /* SDL's ClearInterrupt clears completion only. Also acknowledge a previous
       source bus error, so a damaged slot cannot mask the healthy slot. */
    DMAC->CH[0].unINTR.u32Register = Cy_MDMA_Chnl_GetInterruptCause(DMAC, 0);
    (void)DMAC->CH[0].unINTR.u32Register;
    if (Cy_MDMA_Descr_Init(&descriptor, &transfer) != CY_MDMA_SUCCESS ||
        Cy_MDMA_Chnl_Init(DMAC, 0, &channel) != CY_MDMA_SUCCESS) return WF_IO_ERROR;
    Cy_MDMA_Enable(DMAC);
    Cy_MDMA_Chnl_Enable(DMAC, 0);
    __DMB();
    Cy_MDMA_Chnl_SetSwTrigger(DMAC, 0);
    const uint32_t start = DWT->CYCCNT;
    uint32_t cause;
    do {
        cause = Cy_MDMA_Chnl_GetInterruptCause(DMAC, 0);
    } while (!cause && DWT->CYCCNT - start < SystemCoreClock / 1000u);
    Cy_MDMA_Chnl_Disable(DMAC, 0);
    __DSB();
    if (cause & CY_MDMA_INTRCAUSE_SRC_BUS_ERROR) return WF_INVALID;
    return cause == CY_MDMA_INTRCAUSE_COMPLETION ? WF_OK : WF_IO_ERROR;
}

bool work_flash_erase(unsigned slot)
{
    if (!in_slot(slot, 0, CY_WORK_SES_SIZE_IN_WORD)) return false;
    const cy_stc_flash_erasesector_config_t config = {
        .blocking = CY_FLASH_ERASESECTOR_BLOCKING, .Addr = slots[slot]
    };
    Cy_Flashc_WorkWriteEnable();
    cy_en_flashdrv_status_t status = Cy_Flash_EraseSector(NULL, &config, CY_FLASH_DRIVER_BLOCKING);
    Cy_Flashc_WorkWriteDisable();
    Cy_Flashc_InvalidateFlashCacheBuffer();
    return status == CY_FLASH_DRV_SUCCESS &&
           blank_check(slots[slot], CY_WORK_SES_SIZE_IN_WORD) == CY_FLASH_DRV_SUCCESS;
}

bool work_flash_program(unsigned slot, unsigned offset, uint32_t word)
{
    if (!in_slot(slot, offset, 1)) return false;
    const cy_stc_flash_programrow_config_t config = {
        .blocking = CY_FLASH_PROGRAMROW_BLOCKING, .skipBC = CY_FLASH_PROGRAMROW_BLANK_CHECK,
        .dataSize = CY_FLASH_PROGRAMROW_DATA_SIZE_32BIT, .dataLoc = CY_FLASH_PROGRAMROW_DATA_LOCATION_SRAM,
        .destAddr = &slots[slot][offset], .dataAddr = &word
    };
    Cy_Flashc_WorkWriteEnable();
    cy_en_flashdrv_status_t status = Cy_Flash_ProgramRow(NULL, &config, CY_FLASH_DRIVER_BLOCKING);
    Cy_Flashc_WorkWriteDisable();
    Cy_Flashc_InvalidateFlashCacheBuffer();
    return status == CY_FLASH_DRV_SUCCESS;
}
