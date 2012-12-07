
#include "tcpal_os.h"
#include "tcpal_debug.h"

#include "tcbd_feature.h"
#include "tcbd_api_common.h"
#include "tcbd_drv_component.h"
#include "tcbd_drv_io.h"

#define DEBUG_DUMP_FILE

#if defined(DEBUG_DUMP_FILE)
#include <linux/fs.h>
#include <asm/uaccess.h>
#endif

#define SIZE_TEST_BUFFER (1024*4)

static I08U TestBuffer[SIZE_TEST_BUFFER];

#if defined(DEBUG_DUMP_FILE)
void file_dump(uint8_t *p, uint32_t size, char *path)
{
    struct file *flip = NULL;
    //char *filename = "/data/debug_dump.bin";
    mm_segment_t old_fs;

    if(path == NULL) 
    {
        TcbdDebug(1, "invalid filename! %s\n", path);
        return;
    }

    flip = filp_open(path, O_CREAT | O_RDWR | O_APPEND | O_LARGEFILE, 0);
    if (flip == NULL)
    {
        TcbdDebug(1, "%s open failed\n", path);
        return;
    }

    old_fs=get_fs();
    set_fs(KERNEL_DS);

    if(flip->f_op == NULL || flip->f_op->write == NULL)
    {
        TcbdDebug(1, "%s: File has no file operations registered!\n", __func__);
        filp_close(flip, NULL);
        return;
    }
    if(p == NULL)
    {
        TcbdDebug(1, "Invaild pointer! 0x%X\n", (unsigned int)p);
        return;
    }
    flip->f_op->write(flip, p, size, &flip->f_pos);
    set_fs(old_fs);

    filp_close(flip, NULL);
}
#endif


static void TcbdTestMemoryIo(TcbdDevice_t *_device)
{
    I32S ret = 0;
    static I08U TmpBuffer[SIZE_TEST_BUFFER];

    TcpalMemorySet(TmpBuffer, 0, SIZE_TEST_BUFFER);

    TcbdChangeMemoryView(_device, EpRam0Ram1); 
    ret = TcbdMemWrite(_device, CODE_MEM_BASE, TestBuffer, SIZE_TEST_BUFFER);
    if(ret < 0)
    {
        TcbdDebug(0, "memory write failed %d\n", (int)ret);
        goto exitMemoryTest;
    }

    ret = TcbdMemRead(_device, CODE_MEM_BASE, TmpBuffer, SIZE_TEST_BUFFER);
    if(ret < 0)
    {
        TcbdDebug(0, "memory read failed %d\n", (int)ret);
        goto exitMemoryTest;
    }

#ifdef DEBUG_DUMP_FILE
    file_dump(TestBuffer, SIZE_TEST_BUFFER, "/mnt/sdcard/tflash/org.bin");
    file_dump(TmpBuffer, SIZE_TEST_BUFFER, "/mnt/sdcard/tflash/read.bin");
#endif

    if(TcpalMemoryCompare(TmpBuffer, TestBuffer, SIZE_TEST_BUFFER))
    {
        goto exitMemoryTest;
    }
    TcbdDebug(0, "Memory Read/Write test succeed!\n");
    return;

exitMemoryTest:
        TcbdDebug(0, "Memory Read/Write test failed!\n");
}

static void TcbdTestRegRW(TcbdDevice_t *_device)
{
    I32S ret = 0;
    I08U addr = 0x1B, rdata, wdata = 0xAB;

    ret = TcbdRegRead(_device, addr, &rdata);
    if(ret < 0)
    {
        TcbdDebug(0, "RegRead failed! %d \n", (int)ret);
        goto exitRegRW;
    }
    TcbdDebug(0, "default value : 0x%X\n", rdata);

    TcbdDebug(0, "write 0x%X\n", wdata);
    ret = TcbdRegWrite(_device, addr, wdata);
    if(ret < 0)
    {
        TcbdDebug(0, "RegWrite failed! %d \n", (int)ret);
        goto exitRegRW;
    }

    ret = TcbdRegRead(_device, addr, &rdata);
    if(ret < 0)
    {
        TcbdDebug(0, "RegReadExCon failed! %d \n", (int)ret);
        goto exitRegRW;
    }
    TcbdDebug(0, "single RW :: W[0x%X] == R[0x%X]\n", wdata, rdata);
    return;

exitRegRW:
    TcbdDebug(0, "register burst Write/Read failed!\n");

}


static void TcbdTestRegBurstRWCon(TcbdDevice_t *_device)
{
    I32S ret = 0;
    I08U addr;
    I32U rdata, wdata;

    addr = TCBD_CMDDMA_SADDR;
    wdata = SWAP32(CODE_MEM_BASE);

    ret = TcbdRegReadExCon(_device, addr, (I08U*)&rdata, sizeof(I32U));
    if(ret < 0)
    {
        TcbdDebug(0, "RegReadExCon failed! %d \n", (int)ret);
        goto exitRegBursetRW;
    }
    TcbdDebug(0, "default value : 0x%X\n", (unsigned int)SWAP32(rdata));

    TcbdDebug(0, "write 0x%X\n", (unsigned int)wdata);
    ret = TcbdRegWriteExCon(_device, addr, (I08U*)&wdata, sizeof(I32U));
    if(ret < 0)
    {
        TcbdDebug(0, "RegWriteExCon failed! %d \n", (int)ret);
        goto exitRegBursetRW;
    }

    ret = TcbdRegReadExCon(_device, addr, (I08U*)&rdata, sizeof(I32U));
    if(ret < 0)
    {
        TcbdDebug(0, "RegReadExCon failed! %d \n", (int)ret);
        goto exitRegBursetRW;
    }
    TcbdDebug(0, "multi RW :: W[0x%X] == R[0x%X]\n", (unsigned int)wdata, (unsigned int)SWAP32(rdata));
    return;

exitRegBursetRW:
    TcbdDebug(0, "register burst Write/Read failed!\n");
}

static void TcbdTestBurstRWFix(TcbdDevice_t *_device)
{

}

static void TcbdTestBoot(TcbdDevice_t *_device)
{
    I32S ret = 0;
    I32S ver = 0;
    I08U *bin = TestBuffer;
    I32S size = SIZE_TEST_BUFFER;
    I32U pllData[] = {0x60, 0x00, 0x0F, 0x03, TCBD_DEF_OSCCLK_RF}; 

    ret = TcbdInitPll(_device, pllData);

    ret |= TcbdDownloadBootCode(_device, bin, size);
    if(ret < 0)
    {
        TcbdDebug(0, "failed to boot! %d\n", (int)ret);
        return;
    }
    TcbdDebug(0, "Download success!!\n");

    ret |= TcbdRecvBootCodeVersion(_device, &ver);
    if(ret < 0)
    {
        TcbdDebug(0, "failed to receive boot version! %d \n", (int)ret);
        return;
    }

    TcbdInitDiversityIo(_device, DivIoTypeSingle);

    TcbdDebug(0, "received boot version: 0x%X\n", (unsigned int)ver);
}

static void TcbdTestFrequencySet(TcbdDevice_t *_device)
{

}

void TcbdTestIo(TcbdDevice_t *_device, I32S _testItem)
{
    switch(_testItem)
    {
        case 0: TcbdTestBoot(_device); break;
        case 1: TcbdTestMemoryIo(_device); break;
        case 2: TcbdTestRegBurstRWCon(_device); break;
        case 3: TcbdTestRegRW(_device); break;
        case 4: TcbdTestFrequencySet(_device); break;
        case 5: TcbdTestBurstRWFix(_device); break;
    }
}
