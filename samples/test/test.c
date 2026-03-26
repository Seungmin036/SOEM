/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "soem/soem.h"

static ecx_contextt ctx;
static uint8_t IOmap[4096];

#pragma pack(push, 1)
typedef struct
{
    uint16_t controlword;    /* 0x6040:00, 16 bits */
    int8_t   mode;           /* 0x6060:00, 8 bits  */
    int16_t  target_torque;  /* 0x6071:00, 16 bits */
} rxpdo_t;

typedef struct
{
    uint16_t statusword;      /* 0x6041:00, 16 bits */
    int8_t   mode_display;    /* 0x6061:00, 8 bits  */
    int32_t  position_actual; /* 0x6064:00, 32 bits */
    int32_t  velocity_actual; /* 0x606C:00, 32 bits */
    int16_t  torque_actual;   /* 0x6077:00, 16 bits */
} txpdo_t;
#pragma pack(pop)

static const char *state_to_string(uint16 state)
{
    state &= (uint16)~EC_STATE_ACK;

    switch (state)
    {
    case EC_STATE_INIT:        return "INIT";
    case EC_STATE_PRE_OP:      return "PRE_OP";
    case EC_STATE_BOOT:        return "BOOT";
    case EC_STATE_SAFE_OP:     return "SAFE_OP";
    case EC_STATE_OPERATIONAL: return "OP";
    default:                   return "UNKNOWN";
    }
}

static void print_statusword_bits(uint16_t sw)
{
    printf("  statusword bits   = ");
    for (int b = 15; b >= 0; --b)
    {
        printf("%d", !!(sw & (1u << b)));
    }
    printf("\n");
}

static void drain_errors(void)
{
    while (ctx.ecaterror)
    {
        printf("  %s", ecx_elist2string(&ctx));
    }
}

static int sdo_read_u8(uint16 slave, uint16 index, uint8 subidx, uint8 *value)
{
    int size = sizeof(*value);
    int wkc = ecx_SDOread(&ctx, slave, index, subidx, FALSE, &size, value, EC_TIMEOUTRXM);
    if (wkc <= 0 || size != (int)sizeof(*value))
    {
        return FALSE;
    }
    return TRUE;
}

static int sdo_read_u16(uint16 slave, uint16 index, uint8 subidx, uint16 *value)
{
    int size = sizeof(*value);
    int wkc = ecx_SDOread(&ctx, slave, index, subidx, FALSE, &size, value, EC_TIMEOUTRXM);
    if (wkc <= 0 || size != (int)sizeof(*value))
    {
        return FALSE;
    }
    return TRUE;
}

static int sdo_read_u32(uint16 slave, uint16 index, uint8 subidx, uint32 *value)
{
    int size = sizeof(*value);
    int wkc = ecx_SDOread(&ctx, slave, index, subidx, FALSE, &size, value, EC_TIMEOUTRXM);
    if (wkc <= 0 || size != (int)sizeof(*value))
    {
        return FALSE;
    }
    return TRUE;
}

static void print_sdo_failure(uint16 slave, uint16 index, uint8 subidx)
{
    uint16 state = ctx.slavelist[slave].state;

    printf("0x%04X:%02X read failed on slave %u (state=0x%02X %s)\n",
           index, subidx, slave, state, state_to_string(state));
    drain_errors();
}

static void dump_assign_object(uint16 slave, uint16 index)
{
    uint8 count = 0;

    printf("Reading 0x%04X:00\n", index);

    if (!sdo_read_u8(slave, index, 0x00, &count))
    {
        print_sdo_failure(slave, index, 0x00);
        return;
    }

    printf("0x%04X assignment count: %u\n", index, count);
    for (uint8 sub = 1; sub <= count; ++sub)
    {
        uint16 pdo_index = 0;

        if (!sdo_read_u16(slave, index, sub, &pdo_index))
        {
            print_sdo_failure(slave, index, sub);
            continue;
        }

        printf("  0x%04X:%02X = 0x%04X\n", index, sub, pdo_index);
    }
}

static void dump_map_object(uint16 slave, uint16 index)
{
    uint8 count = 0;

    printf("Reading 0x%04X:00\n", index);

    if (!sdo_read_u8(slave, index, 0x00, &count))
    {
        print_sdo_failure(slave, index, 0x00);
        return;
    }

    printf("0x%04X mapped entry count: %u\n", index, count);
    for (uint8 sub = 1; sub <= count; ++sub)
    {
        uint32 map = 0;
        uint16 obj_index = 0;
        uint8 obj_subidx = 0;
        uint8 bitlen = 0;

        if (!sdo_read_u32(slave, index, sub, &map))
        {
            print_sdo_failure(slave, index, sub);
            continue;
        }

        obj_index = (uint16)((map >> 16) & 0xffffU);
        obj_subidx = (uint8)((map >> 8) & 0xffU);
        bitlen = (uint8)(map & 0xffU);

        printf("  0x%04X:%02X = 0x%08X -> 0x%04X:%02X (%u bits)\n",
               index, sub, map, obj_index, obj_subidx, bitlen);
    }
}

static void dump_fixed_objects(uint16 slave)
{
    static const uint16 assign_indices[] = {0x1C12, 0x1C13};
    static const uint16 rxpdo_maps[] = {0x1703};
    static const uint16 txpdo_maps[] = {0x1B03};

    printf("[PDO assignment]\n");
    for (size_t i = 0; i < sizeof(assign_indices) / sizeof(assign_indices[0]); ++i)
    {
        dump_assign_object(slave, assign_indices[i]);
    }

    printf("\n[RxPDO mapping]\n");
    for (size_t i = 0; i < sizeof(rxpdo_maps) / sizeof(rxpdo_maps[0]); ++i)
    {
        dump_map_object(slave, rxpdo_maps[i]);
    }

    printf("\n[TxPDO mapping]\n");
    for (size_t i = 0; i < sizeof(txpdo_maps) / sizeof(txpdo_maps[0]); ++i)
    {
        dump_map_object(slave, txpdo_maps[i]);
    }
}

static void print_pdo_snapshot(uint16 slave, const txpdo_t *tx)
{
    printf("[PDO snapshot slave %u]\n", slave);
    printf("  statusword      = 0x%04X\n", tx->statusword);
    print_statusword_bits(tx->statusword);
    printf("  mode_display    = %d\n", (int)tx->mode_display);
    printf("  position_actual = %d\n", (int)tx->position_actual);
    printf("  velocity_actual = %d\n", (int)tx->velocity_actual);
    printf("  torque_actual   = %d\n", (int)tx->torque_actual);
}

static void run_cycles(ecx_contextt *ctx_in, uint16 slave, rxpdo_t *rx, txpdo_t *tx,
                       int cycles, const char *label)
{
    (void)slave;
    (void)rx;

    printf("\n[%s]\n", label);

    for (int i = 0; i < cycles; ++i)
    {
        int wkc;
        ecx_send_processdata(ctx_in);
        wkc = ecx_receive_processdata(ctx_in, EC_TIMEOUTRET);

        printf("cycle %02d, wkc=%d\n", i, wkc);
        print_pdo_snapshot(slave, tx);

        osal_usleep(1000);
    }
}

static void read_extra_objects(uint16 slave)
{
    uint8_t  v8  = 0;
    uint16_t err = 0;
    uint32_t din = 0;
    int size;

    size = sizeof(v8);
    if (ecx_SDOread(&ctx, slave, 0x6644, 0x00, FALSE, &size, &v8, EC_TIMEOUTRXM) > 0)
        printf("0x6644 STO status = %u\n", v8);
    else
        printf("read 0x6644 failed\n");

    size = sizeof(v8);
    if (ecx_SDOread(&ctx, slave, 0x6640, 0x00, FALSE, &size, &v8, EC_TIMEOUTRXM) > 0)
        printf("0x6640 STO command = %u\n", v8);
    else
        printf("read 0x6640 failed\n");

    size = sizeof(v8);
    if (ecx_SDOread(&ctx, slave, 0x6633, 0x00, FALSE, &size, &v8, EC_TIMEOUTRXM) > 0)
        printf("0x6633 error acknowledge status = %u\n", v8);
    else
        printf("read 0x6633 failed\n");

    size = sizeof(v8);
    if (ecx_SDOread(&ctx, slave, 0x6632, 0x00, FALSE, &size, &v8, EC_TIMEOUTRXM) > 0)
        printf("0x6632 error acknowledge = %u\n", v8);
    else
        printf("read 0x6632 failed\n");

    size = sizeof(err);
    if (ecx_SDOread(&ctx, slave, 0x603F, 0x00, FALSE, &size, &err, EC_TIMEOUTRXM) > 0)
        printf("0x603F error code = 0x%04X\n", err);
    else
        printf("read 0x603F failed\n");

    size = sizeof(din);
    if (ecx_SDOread(&ctx, slave, 0x60FD, 0x00, FALSE, &size, &din, EC_TIMEOUTRXM) > 0)
        printf("0x60FD digital inputs = 0x%08X\n", din);
    else
        printf("read 0x60FD failed\n");
}

static void sdo_controlword_test(uint16 slave, rxpdo_t *rx, txpdo_t *tx)
{
    uint16 cw;

    printf("\n[SDO controlword test]\n");

    cw = 0x0006;
    rx->controlword = cw;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, sizeof(cw), &cw, EC_TIMEOUTRXM);
    osal_usleep(100000);
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    printf("After SDO CW=0x0006, statusword=0x%04X\n", tx->statusword);

    cw = 0x0007;
    rx->controlword = cw;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, sizeof(cw), &cw, EC_TIMEOUTRXM);
    osal_usleep(100000);
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    printf("After SDO CW=0x0007, statusword=0x%04X\n", tx->statusword);

    cw = 0x000F;
    rx->controlword = cw;
    ecx_SDOwrite(&ctx, slave, 0x6040, 0x00, FALSE, sizeof(cw), &cw, EC_TIMEOUTRXM);
    osal_usleep(100000);
    ecx_send_processdata(&ctx);
    ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
    printf("After SDO CW=0x000F, statusword=0x%04X\n", tx->statusword);
}

int main(int argc, char *argv[])
{
    const char *ifname = NULL;
    uint16 slave = 1;

    rxpdo_t *rx = NULL;
    txpdo_t *tx = NULL;

    if ((argc < 2) || (argc > 3))
    {
        printf("Usage: %s <ifname> [slave]\n", argv[0]);
        return EXIT_FAILURE;
    }

    ifname = argv[1];
    if (argc == 3)
    {
        slave = (uint16)strtoul(argv[2], NULL, 0);
    }

    if (!ecx_init(&ctx, ifname))
    {
        printf("Failed to open interface %s\n", ifname);
        return EXIT_FAILURE;
    }

    if (ecx_config_init(&ctx) <= 0)
    {
        printf("No EtherCAT slaves found\n");
        ecx_close(&ctx);
        return EXIT_FAILURE;
    }

    if ((slave == 0U) || (slave > ctx.slavecount))
    {
        printf("Invalid slave index %u, detected slaves=%d\n", slave, ctx.slavecount);
        ecx_close(&ctx);
        return EXIT_FAILURE;
    }

    printf("Initial master state : 0x%02X %s\n",
           ctx.slavelist[0].state, state_to_string(ctx.slavelist[0].state));
    printf("Initial slave state  : 0x%02X %s\n",
           ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));

    /* PRE-OP */
    ctx.slavelist[0].state = EC_STATE_PRE_OP;
    ecx_writestate(&ctx, 0);
    ecx_statecheck(&ctx, 0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

    if (ctx.slavelist[slave].state != EC_STATE_PRE_OP)
    {
        printf("Slave %u is not PRE_OP yet, requesting directly...\n", slave);
        ctx.slavelist[slave].state = EC_STATE_PRE_OP;
        ecx_writestate(&ctx, slave);
        ecx_statecheck(&ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    }

    printf("After PRE_OP request master state : 0x%02X %s\n",
           ctx.slavelist[0].state, state_to_string(ctx.slavelist[0].state));
    printf("After PRE_OP request slave state  : 0x%02X %s\n\n",
           ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));

    if (ctx.slavelist[slave].state != EC_STATE_PRE_OP)
    {
        printf("ERROR: slave %u failed to reach PRE_OP (0x%02X %s)\n",
               slave, ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));
        ecx_close(&ctx);
        return EXIT_FAILURE;
    }

    dump_fixed_objects(slave);

    printf("Starting config map + SAFE_OP/OP state transitions\n");

    for (int i = 1; i <= ctx.slavecount; ++i)
    {
        ctx.slavelist[i].blockLRW = 1;
        printf("slave %d blockLRW = %d\n", i, ctx.slavelist[i].blockLRW);
    }

    if (!ecx_config_map_group(&ctx, IOmap, 0))
    {
        printf("ecx_config_map_group failed\n");
        ecx_close(&ctx);
        return EXIT_FAILURE;
    }

    /* 처음엔 DC 없이 확인 */
    /* ecx_configdc(&ctx); */

    rx = (rxpdo_t *)ctx.slavelist[slave].outputs;
    tx = (txpdo_t *)ctx.slavelist[slave].inputs;

    if (rx == NULL || tx == NULL)
    {
        printf("PDO pointer binding failed: outputs=%p inputs=%p\n", (void *)rx, (void *)tx);
        ecx_close(&ctx);
        return EXIT_FAILURE;
    }

    printf("Bound PDO pointers: outputs=%p inputs=%p Obytes=%u Ibytes=%u\n",
           (void *)rx, (void *)tx,
           ctx.slavelist[slave].Obytes,
           ctx.slavelist[slave].Ibytes);

    /* SAFE-OP */
    ctx.slavelist[0].state = EC_STATE_SAFE_OP;
    ctx.slavelist[slave].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ctx, 0);
    ecx_writestate(&ctx, slave);
    ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    ecx_statecheck(&ctx, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

    printf("After SAFE_OP request master state : 0x%02X %s\n",
           ctx.slavelist[0].state, state_to_string(ctx.slavelist[0].state));
    printf("After SAFE_OP request slave state  : 0x%02X %s\n",
           ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));

    if ((ctx.slavelist[slave].state & (uint16)~EC_STATE_ACK) != EC_STATE_SAFE_OP)
    {
        printf("ERROR: slave %u failed to reach SAFE_OP\n", slave);
        ecx_close(&ctx);
        return EXIT_FAILURE;
    }

    printf("SAFE_OP OK\n");

    /* OP 전에 유효한 RxPDO 값 채우기 */
    rx->controlword   = 0x0006;
    rx->mode          = 10;  /* CST */
    rx->target_torque = 0;

    printf("Prepared RxPDO before OP request:\n");
    printf("  controlword   = 0x%04X\n", rx->controlword);
    printf("  mode          = %d\n", (int)rx->mode);
    printf("  target_torque = %d\n", (int)rx->target_torque);

    printf("Sending process data before OP request...\n");
    for (int i = 0; i < 200; ++i)
    {
        int wkc;
        ecx_send_processdata(&ctx);
        wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

        if ((i % 50) == 0)
        {
            printf("  pre-OP cycle %d, wkc=%d\n", i, wkc);
        }

        osal_usleep(1000);
    }

    /* OP 요청 */
    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ctx.slavelist[slave].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    ecx_writestate(&ctx, slave);

    /* OP로 넘어가는 동안에도 process data 유지 */
    for (int i = 0; i < 500; ++i)
    {
        int wkc;
        ecx_send_processdata(&ctx);
        wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
        ecx_readstate(&ctx);

        if ((i % 50) == 0)
        {
            printf("  OP transition cycle %d, wkc=%d, master=0x%02X, slave=0x%02X\n",
                   i, wkc, ctx.slavelist[0].state, ctx.slavelist[slave].state);
        }

        if (((ctx.slavelist[0].state & (uint16)~EC_STATE_ACK) == EC_STATE_OPERATIONAL) &&
            ((ctx.slavelist[slave].state & (uint16)~EC_STATE_ACK) == EC_STATE_OPERATIONAL))
        {
            break;
        }

        osal_usleep(1000);
    }

    ecx_readstate(&ctx);

    printf("After OP request master state : 0x%02X %s\n",
           ctx.slavelist[0].state, state_to_string(ctx.slavelist[0].state));
    printf("After OP request slave state  : 0x%02X %s\n",
           ctx.slavelist[slave].state, state_to_string(ctx.slavelist[slave].state));
    printf("AL status code : 0x%04X\n", ctx.slavelist[slave].ALstatuscode);
    printf("AL status text : %s\n", ec_ALstatuscode2string(ctx.slavelist[slave].ALstatuscode));

    if ((ctx.slavelist[slave].state & (uint16)~EC_STATE_ACK) == EC_STATE_OPERATIONAL)
    {
        printf("OP reached. Reading TxPDO...\n");

        for (int i = 0; i < 50; ++i)
        {
            int wkc;
            ecx_send_processdata(&ctx);
            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

            printf("cycle %02d, wkc=%d\n", i, wkc);
            print_pdo_snapshot(slave, tx);

            osal_usleep(1000);
        }

        read_extra_objects(slave);

        sdo_controlword_test(slave, rx, tx);

        /* PDO로만 staged test */
        rx->mode = 10;
        rx->target_torque = 0;

        rx->controlword = 0x0080;
        run_cycles(&ctx, slave, rx, tx, 100, "CW=0x0080");

        rx->controlword = 0x0006;
        run_cycles(&ctx, slave, rx, tx, 100, "CW=0x0006");

        rx->controlword = 0x0007;
        run_cycles(&ctx, slave, rx, tx, 100, "CW=0x0007");

        rx->controlword = 0x000F;
        run_cycles(&ctx, slave, rx, tx, 100, "CW=0x000F");
    }
    else
    {
        printf("Still not OP. Most likely next things to check are:\n");
        printf("  1) drive-specific OP entry condition\n");
        printf("  2) need for DC/sync configuration\n");
        printf("  3) whether controlword/mode must be staged differently\n");
    }

    ecx_close(&ctx);
    return EXIT_SUCCESS;
}